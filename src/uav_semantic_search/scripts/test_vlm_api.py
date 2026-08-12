#!/usr/bin/env python3
"""Diagnose an OpenAI-compatible VLM endpoint used by uav_semantic_search.

The script intentionally uses urllib.request, matching vlm_common.py in the
project. It never writes the API key to the report or terminal.
"""

import argparse
import base64
import json
import os
import socket
import ssl
import statistics
import struct
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import zlib
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def elapsed(start: float) -> float:
    return round(time.perf_counter() - start, 4)


def mask_proxy(value: str) -> str:
    try:
        parsed = urllib.parse.urlsplit(value)
        host = parsed.hostname or ""
        port = (":" + str(parsed.port)) if parsed.port else ""
        return "%s://%s%s" % (parsed.scheme, host, port)
    except Exception:
        return "(configured)"


def load_yaml_config(path: Path) -> Dict[str, Any]:
    try:
        import yaml  # type: ignore
    except ImportError:
        raise RuntimeError(
            "PyYAML is required. On Ubuntu run: sudo apt install python3-yaml"
        )
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    root = raw.get("vlm_semantic_search", raw)
    if not isinstance(root, dict):
        raise RuntimeError("Invalid YAML: vlm_semantic_search must be a mapping.")
    return root


def find_default_config() -> Optional[Path]:
    candidates = [
        Path(__file__).resolve().parent.parent / "config" / "vlm_semantic_search.yaml",
        Path.home()
        / "harp_sar_ws"
        / "src"
        / "uav_semantic_search"
        / "config"
        / "vlm_semantic_search.yaml",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def make_red_png(width: int = 128, height: int = 128) -> bytes:
    def chunk(kind: bytes, data: bytes) -> bytes:
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(
            ">I", zlib.crc32(body) & 0xFFFFFFFF
        )

    # Each scanline starts with PNG filter byte 0.
    row = b"\x00" + (b"\xff\x20\x20" * width)
    raw = row * height
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )


def image_data_url(path: Optional[Path]) -> Tuple[str, int, str]:
    if path is None:
        data = make_red_png()
        mime = "image/png"
        source = "generated_128x128_red_png"
    else:
        data = path.read_bytes()
        suffix = path.suffix.lower()
        mime = {
            ".jpg": "image/jpeg",
            ".jpeg": "image/jpeg",
            ".png": "image/png",
            ".webp": "image/webp",
        }.get(suffix, "application/octet-stream")
        source = str(path)
    encoded = base64.b64encode(data).decode("ascii")
    return "data:%s;base64,%s" % (mime, encoded), len(data), source


def endpoint_from_base(base_url: str) -> str:
    base = base_url.rstrip("/")
    return base if base.endswith("/chat/completions") else base + "/chat/completions"


def classify_exception(exc: BaseException) -> Tuple[str, str]:
    reason = getattr(exc, "reason", exc)
    text = repr(reason)
    if isinstance(reason, socket.gaierror):
        return "DNS_ERROR", text
    if isinstance(reason, ssl.SSLError):
        return "TLS_ERROR", text
    if isinstance(reason, (socket.timeout, TimeoutError)) or "timed out" in text.lower():
        return "TIMEOUT", text
    if isinstance(reason, ConnectionRefusedError):
        return "CONNECTION_REFUSED", text
    if isinstance(reason, OSError):
        return "NETWORK_OS_ERROR", text
    return "CLIENT_ERROR", text


def probe_network(host: str, port: int, timeout: float) -> Dict[str, Any]:
    result: Dict[str, Any] = {"host": host, "port": port}
    addresses: List[str] = []

    started = time.perf_counter()
    try:
        infos = socket.getaddrinfo(host, port, type=socket.SOCK_STREAM)
        addresses = list(dict.fromkeys(item[4][0] for item in infos))
        result.update(
            dns_ok=True,
            dns_sec=elapsed(started),
            resolved_addresses=addresses,
        )
    except Exception as exc:
        category, detail = classify_exception(exc)
        result.update(
            dns_ok=False,
            dns_sec=elapsed(started),
            failure_category=category,
            error=detail,
        )
        return result

    started = time.perf_counter()
    raw_sock = None
    try:
        raw_sock = socket.create_connection((host, port), timeout=timeout)
        result.update(tcp_ok=True, tcp_sec=elapsed(started))
    except Exception as exc:
        category, detail = classify_exception(exc)
        result.update(
            tcp_ok=False,
            tcp_sec=elapsed(started),
            failure_category=category,
            error=detail,
        )
        return result

    if port == 443:
        started = time.perf_counter()
        try:
            context = ssl.create_default_context()
            with context.wrap_socket(raw_sock, server_hostname=host) as tls_sock:
                cert = tls_sock.getpeercert()
                result.update(
                    tls_ok=True,
                    tls_sec=elapsed(started),
                    tls_version=tls_sock.version(),
                    certificate_expires=cert.get("notAfter"),
                )
                raw_sock = None
        except Exception as exc:
            category, detail = classify_exception(exc)
            result.update(
                tls_ok=False,
                tls_sec=elapsed(started),
                failure_category=category,
                error=detail,
            )
        finally:
            if raw_sock is not None:
                raw_sock.close()
    else:
        raw_sock.close()
        result["tls_skipped"] = True
    return result


def build_payload(model: str, mode: str, image_url: Optional[str]) -> Dict[str, Any]:
    if mode == "text":
        user_content: Any = (
            'Return only this JSON object with no Markdown: {"status":"ok","test":"text"}'
        )
    else:
        user_content = [
            {
                "type": "text",
                "text": (
                    "Inspect the image. Return only a JSON object containing "
                    'keys "status", "test", and "dominant_color".'
                ),
            },
            {"type": "image_url", "image_url": {"url": image_url}},
        ]
    return {
        "model": model,
        "temperature": 0.0,
        "max_tokens": 80,
        "messages": [
            {
                "role": "system",
                "content": "You are an API health-check. Output valid compact JSON only.",
            },
            {"role": "user", "content": user_content},
        ],
    }


def http_category(status: int) -> str:
    if status in (401, 403):
        return "AUTH_OR_PERMISSION_ERROR"
    if status == 404:
        return "ENDPOINT_OR_MODEL_NOT_FOUND"
    if status == 408:
        return "UPSTREAM_TIMEOUT"
    if status == 429:
        return "RATE_LIMIT_OR_CAPACITY"
    if 500 <= status:
        return "API_SERVER_ERROR"
    return "HTTP_ERROR"


def call_api(
    endpoint: str,
    api_key: str,
    payload: Dict[str, Any],
    timeout: float,
    mode: str,
    run_index: int,
) -> Dict[str, Any]:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    request = urllib.request.Request(
        endpoint,
        data=body,
        headers={
            "Content-Type": "application/json",
            **({"Authorization": "Bearer " + api_key} if api_key else {}),
        },
        method="POST",
    )
    result: Dict[str, Any] = {
        "mode": mode,
        "run": run_index,
        "started_at": now_iso(),
        "timeout_sec": timeout,
        "request_bytes": len(body),
    }
    overall = time.perf_counter()
    try:
        opened = time.perf_counter()
        with urllib.request.urlopen(request, timeout=timeout) as response:
            result["time_to_headers_sec"] = elapsed(opened)
            body_started = time.perf_counter()
            raw = response.read()
            result["body_read_sec"] = elapsed(body_started)
            result["http_status"] = int(response.status)
            result["response_bytes"] = len(raw)
        result["total_sec"] = elapsed(overall)
        decoded = raw.decode("utf-8", errors="replace")
        try:
            parsed = json.loads(decoded)
            choices = parsed.get("choices", [])
            content = (
                choices[0].get("message", {}).get("content", "")
                if choices and isinstance(choices[0], dict)
                else ""
            )
            if parsed.get("error"):
                result.update(
                    ok=False,
                    failure_category="API_ERROR_IN_200_RESPONSE",
                    error=str(parsed["error"])[:500],
                )
            elif not choices:
                result.update(
                    ok=False,
                    failure_category="INVALID_API_SCHEMA",
                    error="HTTP 200 response has no choices.",
                    response_preview=decoded[:300],
                )
            else:
                result.update(
                    ok=True,
                    response_preview=str(content)[:300],
                )
                usage = parsed.get("usage")
                if isinstance(usage, dict):
                    result["usage"] = usage
        except json.JSONDecodeError as exc:
            result.update(
                ok=False,
                failure_category="INVALID_JSON_RESPONSE",
                error=repr(exc),
                response_preview=decoded[:300],
            )
    except urllib.error.HTTPError as exc:
        raw_error = exc.read().decode("utf-8", errors="replace")
        result.update(
            ok=False,
            http_status=int(exc.code),
            total_sec=elapsed(overall),
            failure_category=http_category(int(exc.code)),
            error=raw_error[:500],
        )
    except Exception as exc:
        category, detail = classify_exception(exc)
        result.update(
            ok=False,
            total_sec=elapsed(overall),
            failure_category=category,
            error=detail,
        )
    return result


def mode_summary(items: List[Dict[str, Any]]) -> Dict[str, Any]:
    successful = [item for item in items if item.get("ok")]
    totals = [float(item["total_sec"]) for item in successful if "total_sec" in item]
    categories: Dict[str, int] = {}
    for item in items:
        if item.get("ok"):
            continue
        key = str(item.get("failure_category", "UNKNOWN"))
        categories[key] = categories.get(key, 0) + 1
    summary: Dict[str, Any] = {
        "attempts": len(items),
        "successes": len(successful),
        "success_rate_percent": round(100.0 * len(successful) / max(1, len(items)), 1),
        "failure_counts": categories,
    }
    if totals:
        summary.update(
            latency_min_sec=round(min(totals), 3),
            latency_mean_sec=round(statistics.mean(totals), 3),
            latency_median_sec=round(statistics.median(totals), 3),
            latency_max_sec=round(max(totals), 3),
        )
        if len(totals) > 1:
            summary["latency_stdev_sec"] = round(statistics.stdev(totals), 3)
    return summary


def diagnosis(
    network: Dict[str, Any],
    summaries: Dict[str, Dict[str, Any]],
    results: List[Dict[str, Any]],
) -> List[str]:
    notes: List[str] = []
    if not network.get("dns_ok"):
        return ["DNS 解析失败，优先检查本机 DNS、代理或网络连接。"]
    if not network.get("tcp_ok"):
        return ["DNS 正常但 TCP 连接失败，优先检查路由、防火墙、代理或服务端端口。"]
    if network.get("port") == 443 and not network.get("tls_ok"):
        return ["TCP 正常但 TLS 握手失败，优先检查证书、系统时间、代理或 HTTPS 劫持。"]

    failures = {
        str(item.get("failure_category"))
        for item in results
        if not item.get("ok")
    }
    if "AUTH_OR_PERMISSION_ERROR" in failures:
        notes.append("接口返回 401/403：API 密钥、账户权限或模型权限有问题，不是网络超时。")
    if "ENDPOINT_OR_MODEL_NOT_FOUND" in failures:
        notes.append("接口返回 404：检查 base_url、/chat/completions 路径和模型名称。")
    if "RATE_LIMIT_OR_CAPACITY" in failures:
        notes.append("接口返回 429：属于限流、余额或服务容量问题，增加客户端超时通常无效。")
    if "API_SERVER_ERROR" in failures or "UPSTREAM_TIMEOUT" in failures:
        notes.append("接口返回 5xx/408：服务端或上游模型异常，应联系 API 服务商并保留报告时间。")
    if "TIMEOUT" in failures:
        notes.append(
            "DNS/TCP/TLS 正常而正式请求超时：更可能是 API 排队、模型推理慢或响应体生成慢；"
            "也可能是链路在长连接期间不稳定。"
        )

    text = summaries.get("text", {})
    image = summaries.get("image", {})
    if text and image:
        if text.get("success_rate_percent") == 100.0 and image.get("success_rate_percent", 0) < 100.0:
            notes.append(
                "文本请求稳定但图像请求失败：重点检查视觉模型支持、图像大小/编码和服务端视觉处理能力。"
            )
        elif text.get("success_rate_percent") == 100.0 and image.get("success_rate_percent") == 100.0:
            text_mean = float(text.get("latency_mean_sec", 0.0))
            image_mean = float(image.get("latency_mean_sec", 0.0))
            if image_mean > max(5.0, text_mean * 2.0):
                notes.append("图像请求明显慢于文本请求，当前超时主要受视觉推理或图像上传影响。")

    if all(value.get("success_rate_percent") == 100.0 for value in summaries.values()):
        notes.append(
            "本次独立 API 测试全部成功。若 ROS 中仍超时，应继续检查真实相机图像体积、"
            "三台机器人串行请求、重试总期限与协调器等待时间是否匹配。"
        )
    if not notes:
        notes.append("请根据 failure_category 和各阶段耗时进一步定位；建议在故障时连续测试 10 次。")
    return notes


def print_result(item: Dict[str, Any]) -> None:
    if item.get("ok"):
        print(
            "[%s #%d] OK  total=%.2fs headers=%.2fs request=%dB response=%dB"
            % (
                item["mode"],
                item["run"],
                item.get("total_sec", 0.0),
                item.get("time_to_headers_sec", 0.0),
                item.get("request_bytes", 0),
                item.get("response_bytes", 0),
            )
        )
    else:
        print(
            "[%s #%d] FAIL  type=%s status=%s total=%.2fs"
            % (
                item["mode"],
                item["run"],
                item.get("failure_category", "UNKNOWN"),
                item.get("http_status", "-"),
                item.get("total_sec", 0.0),
            )
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Test DNS/TCP/TLS and repeated OpenAI-compatible text/VLM requests."
    )
    parser.add_argument("--config", type=Path, help="Path to vlm_semantic_search.yaml")
    parser.add_argument("--base-url", help="Override backend.base_url")
    parser.add_argument("--model", help="Override backend.model")
    parser.add_argument(
        "--api-key-env",
        help="Read key from this environment variable instead of YAML/default key variable.",
    )
    parser.add_argument("--timeout", type=float, default=None, help="Per-request timeout in seconds")
    parser.add_argument("--runs", type=int, default=5, help="Requests per selected mode")
    parser.add_argument(
        "--mode", choices=("text", "image", "both"), default="both"
    )
    parser.add_argument(
        "--image",
        type=Path,
        help="Real JPG/PNG/WebP image. Without this, a small generated PNG is used.",
    )
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between calls")
    parser.add_argument("--output", type=Path, help="JSON report path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config_path = args.config or find_default_config()
    if config_path is None or not config_path.is_file():
        print("Cannot find vlm_semantic_search.yaml; pass --config PATH.", file=sys.stderr)
        return 2

    try:
        root = load_yaml_config(config_path)
        backend = dict(root.get("backend") or {})
        base_url = str(args.base_url or backend.get("base_url") or "").strip()
        model = str(args.model or backend.get("model") or "").strip()
        if not base_url or not model:
            raise RuntimeError("backend.base_url and backend.model are required.")
        key_env = str(
            args.api_key_env or backend.get("api_key_env") or "VLM_API_KEY"
        )
        # Match current project behavior: direct YAML key takes precedence unless
        # the user explicitly selected an environment variable on the CLI.
        if args.api_key_env:
            api_key = os.environ.get(key_env, "").strip()
            key_source = "environment:" + key_env
        else:
            direct_key = str(backend.get("api_key") or "").strip()
            api_key = direct_key or os.environ.get(key_env, "").strip()
            key_source = "yaml" if direct_key else "environment:" + key_env
        timeout = float(args.timeout or backend.get("timeout_sec") or 45.0)
        if timeout <= 0 or args.runs <= 0:
            raise RuntimeError("--timeout and --runs must be positive.")
        if not api_key:
            raise RuntimeError(
                "No API key found. Set %s or configure backend.api_key." % key_env
            )
        endpoint = endpoint_from_base(base_url)
        parsed = urllib.parse.urlsplit(endpoint)
        if parsed.scheme not in ("http", "https") or not parsed.hostname:
            raise RuntimeError("Invalid endpoint URL: %s" % endpoint)
        port = parsed.port or (443 if parsed.scheme == "https" else 80)
        if args.image and not args.image.is_file():
            raise RuntimeError("Image does not exist: %s" % args.image)
    except Exception as exc:
        print("Configuration error: %s" % exc, file=sys.stderr)
        return 2

    report_path = args.output or Path(
        "vlm_api_diagnostic_%s.json" % datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    image_url, image_bytes, image_source = image_data_url(args.image)
    proxies = {
        key: mask_proxy(value)
        for key, value in urllib.request.getproxies().items()
        if value
    }

    print("VLM API diagnostic")
    print("  endpoint :", endpoint)
    print("  model    :", model)
    print("  key      :", key_source, "(value hidden)")
    print("  timeout  :", "%.1fs" % timeout)
    print("  runs     :", args.runs, "per mode")
    print("  proxies  :", proxies or "none detected")
    if args.mode in ("image", "both"):
        print("  image    :", image_source, "(%d bytes before base64)" % image_bytes)

    print("\n[1/2] Network phases")
    network = probe_network(parsed.hostname, port, min(timeout, 15.0))
    print(json.dumps(network, indent=2, ensure_ascii=False))

    print("\n[2/2] API requests")
    selected_modes = ["text", "image"] if args.mode == "both" else [args.mode]
    results: List[Dict[str, Any]] = []
    for mode in selected_modes:
        payload = build_payload(model, mode, image_url if mode == "image" else None)
        for run_index in range(1, args.runs + 1):
            item = call_api(
                endpoint, api_key, payload, timeout, mode, run_index
            )
            results.append(item)
            print_result(item)
            if args.interval > 0 and not (
                mode == selected_modes[-1] and run_index == args.runs
            ):
                time.sleep(args.interval)

    summaries = {
        mode: mode_summary([item for item in results if item["mode"] == mode])
        for mode in selected_modes
    }
    notes = diagnosis(network, summaries, results)
    report = {
        "created_at": now_iso(),
        "config_path": str(config_path),
        "endpoint": endpoint,
        "model": model,
        "api_key_source": key_source,
        "api_key_included_in_report": False,
        "timeout_sec": timeout,
        "runs_per_mode": args.runs,
        "proxy_configuration": proxies,
        "image_source": image_source if "image" in selected_modes else None,
        "image_bytes_before_base64": image_bytes if "image" in selected_modes else None,
        "network_probe": network,
        "summary": summaries,
        "diagnosis": notes,
        "requests": results,
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    print("\nSummary")
    print(json.dumps(summaries, indent=2, ensure_ascii=False))
    print("\nDiagnosis")
    for note in notes:
        print(" -", note)
    print("\nReport:", report_path.resolve())
    return 0 if all(item.get("ok") for item in results) else 1


if __name__ == "__main__":
    sys.exit(main())
