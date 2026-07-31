from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import time
import urllib.error
import urllib.request
from typing import Any
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from fastmcp import Client


DEFAULT_MCP_URL = "http://127.0.0.1:8081/mcp"
DEFAULT_BASE_URL = "https://api.openai.com/v1"
DEFAULT_MODEL = "gpt-4.1-mini"
MOTION_TOOLS = {
    "safe_home",
    "gravity_compensation_start",
    "set_gripper_opening_mm",
    "move_to_pose",
    "move_joints",
    "pick_color",
    "record_replay",
}

TOOL_CATEGORIES = {
    "get_robot_status": ("状态与诊断", "#5fa8ff"),
    "diagnose_ros": ("状态与诊断", "#5fa8ff"),
    "enable_robot": ("使能控制", "#77c96b"),
    "disable_robot": ("使能控制", "#77c96b"),
    "safe_home": ("运动控制", "#33d6b0"),
    "move_to_pose": ("运动控制", "#33d6b0"),
    "move_joints": ("运动控制", "#33d6b0"),
    "ik_check": ("运动控制", "#33d6b0"),
    "set_gripper_opening_mm": ("夹爪控制", "#f2a541"),
    "gravity_compensation_status": ("重力补偿", "#a78bfa"),
    "gravity_compensation_start": ("重力补偿", "#a78bfa"),
    "gravity_compensation_stop": ("重力补偿", "#a78bfa"),
    "detect_blocks": ("视觉抓取", "#ef5a4d"),
    "pick_color": ("视觉抓取", "#ef5a4d"),
    "record_start": ("录制回放", "#e879f9"),
    "record_stop": ("录制回放", "#e879f9"),
    "record_replay": ("录制回放", "#e879f9"),
    "record_clear": ("录制回放", "#e879f9"),
}

SYSTEM_PROMPT = """你是 reBotArm 机械臂的智能控制助手。你必须使用 MCP tools 执行用户的指令，而不是解释如何执行。

## 核心规则：
1. **直接执行，不要解释**：用户说"摆姿势"就调用 move_to_pose，说"抓红色"就调用 pick_color，不要输出步骤说明或教程。
2. **使用工具获取真实信息**：用户问状态、问看到什么、问色块位置，必须调用 get_robot_status 或 detect_blocks，禁止编造数据。
3. **参数必须合理**：move_to_pose 的 x 在 [-0.4, 0.4] 之间，y 在 [-0.3, 0.3] 之间，z 在 [0.1, 0.5] 之间。
4. **抓取流程**：明确颜色时直接 pick_color；颜色不明确时先 detect_blocks。
5. **安全第一**：未知的目标或危险操作先询问用户确认。
6. **随机生成**：用户要求"摆姿势"、"动一动"等非具体指令时，每次生成不同的随机坐标，不要重复使用相同的数值。

## 可用工具：
- get_robot_status: 获取机械臂状态
- diagnose_ros: 诊断 ROS 连接
- enable_robot / disable_robot: 启用/禁用机械臂
- safe_home: 回到安全位置
- move_to_pose: 移动到指定位置（x, y, z, roll_deg, pitch_deg, yaw_deg, duration）
- move_joints: 控制关节角度
- set_gripper_opening_mm: 设置夹爪开度（0-90mm）
- detect_blocks: 检测颜色物块
- pick_color: 抓取指定颜色物块
- record_start / record_stop / record_replay: 录制/重放动作

## 回复格式：
- 直接调用工具，输出尽量少的自然语言，最多一句话说明你的操作。
- 不要输出数学公式、方框、代码块或教程。
"""


class ChatCompletionsLLM:
    def __init__(
        self,
        *,
        base_url: str,
        api_key: str,
        model: str,
        timeout_sec: float,
        temperature: float,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.api_key = api_key or "local"
        self.model = model
        self.timeout_sec = max(float(timeout_sec), 5.0)
        self.temperature = float(temperature)

    def complete(
        self,
        messages: list[dict[str, Any]],
        tools: list[dict[str, Any]],
    ) -> dict[str, Any]:
        payload = {
            "model": self.model,
            "messages": messages,
            "tools": tools,
            "tool_choice": "auto",
            "temperature": self.temperature,
        }
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        request = urllib.request.Request(
            f"{self.base_url}/chat/completions",
            data=body,
            headers={
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"LLM HTTP {exc.code}: {detail}") from exc
        except urllib.error.URLError as exc:
            raise RuntimeError(f"LLM request failed: {exc}") from exc


async def run_repl(args: argparse.Namespace) -> int:
    llm = ChatCompletionsLLM(
        base_url=args.base_url,
        api_key=args.api_key,
        model=args.model,
        timeout_sec=args.timeout_sec,
        temperature=args.temperature,
    )

    async with Client(args.mcp_url) as mcp:
        mcp_tools = await mcp.list_tools()
        tools = [_mcp_tool_to_chat_tool(tool) for tool in mcp_tools]
        tool_names = [tool["function"]["name"] for tool in tools]
        messages: list[dict[str, Any]] = [{"role": "system", "content": SYSTEM_PROMPT}]

        print(f"Connected MCP: {args.mcp_url}")
        print(f"LLM model: {args.model}")
        print(f"Tools: {', '.join(tool_names)}")
        print(
            "输入中文指令；/tools 查看工具，/status 诊断，/detect 查看色块，"
            "/pick red 抓取，/gripper 90 控制夹爪，/reset 清空上下文，/exit 退出。"
            "在本地命令后加 --json 可显示原始数据。"
        )

        while True:
            try:
                user_text = input("\n你 > ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                return 0

            if not user_text:
                continue
            if user_text in {"/exit", "/quit", "exit", "quit"}:
                return 0
            if user_text == "/tools":
                print(", ".join(tool_names))
                continue
            if user_text == "/reset":
                messages = [{"role": "system", "content": SYSTEM_PROMPT}]
                print("上下文已清空。")
                continue
            if user_text == "/status":
                result = await mcp.call_tool("diagnose_ros", {})
                print(_compact_json(_mcp_result_to_json(result), limit=args.result_chars))
                continue
            if await _try_local_command(
                mcp,
                user_text,
                confirm_motion=not args.yes,
                result_chars=args.result_chars,
                verbose_tools=args.verbose_tools,
            ):
                continue

            messages.append({"role": "user", "content": user_text})
            await _run_agent_turn(
                llm,
                mcp,
                messages,
                tools,
                confirm_motion=not args.yes,
                max_rounds=args.max_tool_rounds,
                result_chars=args.result_chars,
            )


async def _run_agent_turn(
    llm: ChatCompletionsLLM,
    mcp: Client,
    messages: list[dict[str, Any]],
    tools: list[dict[str, Any]],
    *,
    confirm_motion: bool,
    max_rounds: int,
    result_chars: int,
) -> None:
    for _ in range(max(1, int(max_rounds))):
        try:
            response = llm.complete(messages, tools)
        except Exception as exc:
            print(f"\n助手 > LLM 请求失败：{exc}")
            print("提示：先检查 VM 的 DNS/网络；MCP 本地工具仍可用，例如 /status、/detect、/pick red。")
            messages.append(
                {
                    "role": "assistant",
                    "content": f"LLM request failed: {exc}",
                }
            )
            return
        choice = (response.get("choices") or [{}])[0]
        message = choice.get("message") or {}
        content = message.get("content") or ""
        tool_calls = message.get("tool_calls") or []

        if content:
            print(f"\n助手 > {content}")

        if not tool_calls:
            parsed_calls = _parse_tool_calls_from_text(content)
            if parsed_calls:
                tool_calls = parsed_calls
            else:
                messages.append({"role": "assistant", "content": content})
                return

        assistant_message = {"role": "assistant", "content": content, "tool_calls": tool_calls}
        messages.append(assistant_message)

        for call in tool_calls:
            function = call.get("function") or {}
            name = str(function.get("name") or "")
            arguments = _parse_tool_arguments(function.get("arguments"), name)
            tool_call_id = str(call.get("id") or f"tool-{int(time.time() * 1000)}")

            if not name:
                tool_result = {"ok": False, "message": "LLM emitted a tool call without a name."}
            elif confirm_motion and name in MOTION_TOOLS:
                if not _confirm_tool(name, arguments):
                    tool_result = {
                        "ok": False,
                        "tool": name,
                        "cancelled": True,
                        "message": "User declined this motion tool call.",
                    }
                else:
                    tool_result = await _call_mcp_tool(mcp, name, arguments)
            else:
                tool_result = await _call_mcp_tool(mcp, name, arguments)

            print(f"\n工具 > {name}({_compact_json(arguments, limit=260)})")
            print(f"结果 > {_compact_json(tool_result, limit=result_chars)}")
            messages.append(
                {
                    "role": "tool",
                    "tool_call_id": tool_call_id,
                    "content": _compact_json(tool_result, limit=result_chars),
                }
            )

    print("\n助手 > 工具调用轮次已到上限，我先停在这里。")


async def _try_local_command(
    mcp: Client,
    text: str,
    *,
    confirm_motion: bool,
    result_chars: int,
    verbose_tools: bool,
) -> bool:
    parts = text.split()
    if not parts:
        return False
    command = parts[0].lower()
    show_json = "--json" in parts or verbose_tools
    parts = [part for part in parts if part != "--json"]
    intent = _parse_builtin_intent(text)
    if intent is not None:
        command = intent["command"]
        parts = [command, *intent.get("args", [])]
        show_json = show_json or verbose_tools

    if command == "/detect":
        color = parts[1] if len(parts) > 1 else "auto"
        result = await _call_mcp_tool(mcp, "detect_blocks", {"preferred_color": color})
        print(_summarize_detect_blocks(result))
    elif command == "/pick":
        color = parts[1] if len(parts) > 1 else "auto"
        arguments = {"color": color}
        if confirm_motion and not _confirm_tool("pick_color", arguments):
            result = {"ok": False, "tool": "pick_color", "cancelled": True}
        else:
            result = await _call_mcp_tool(mcp, "pick_color", arguments)
        print(_summarize_pick_color(result))
    elif command == "/gripper":
        if len(parts) < 2:
            print("用法：/gripper 90")
            return True
        try:
            opening_mm = float(parts[1])
        except ValueError:
            print("夹爪开度需要是数字，单位 mm。")
            return True
        arguments = {"opening_mm": opening_mm}
        if confirm_motion and not _confirm_tool("set_gripper_opening_mm", arguments):
            result = {"ok": False, "tool": "set_gripper_opening_mm", "cancelled": True}
        else:
            result = await _call_mcp_tool(mcp, "set_gripper_opening_mm", arguments)
        print(_summarize_gripper(result))
    elif command == "/pose":
        arguments = _generate_random_pose()
        if confirm_motion and not _confirm_tool("move_to_pose", arguments):
            result = {"ok": False, "tool": "move_to_pose", "cancelled": True}
        else:
            result = await _call_mcp_tool_with_retry(mcp, "move_to_pose", arguments, max_retries=3)
        print(f"助手 > 正在摆姿势...")
        print(f"工具 > move_to_pose({_compact_json(arguments, limit=200)})")
        print(_summarize_pose(result))
        if not result.get("ok"):
            print(f"结果 > {_compact_json(result, limit=2000)}")
    else:
        return False

    if show_json:
        print(_compact_json(result, limit=result_chars))
    return True


def _parse_builtin_intent(text: str) -> dict[str, Any] | None:
    normalized = text.strip().lower()
    compact = "".join(normalized.split())
    color = _extract_color(compact)

    if any(
        token in compact
        for token in (
            "看到哪些",
            "看到了哪些",
            "能看到什么",
            "看到什么",
            "有哪些颜色",
            "哪些颜色",
            "色块",
            "检测",
        )
    ):
        return {"command": "/detect", "args": [color] if color else []}

    if any(token in compact for token in ("抓取", "抓一下", "夹取", "拿起", "抓住", "抓", "捡", "pick")):
        return {"command": "/pick", "args": [color or "auto"]}

    if "打开夹爪" in compact or "张开夹爪" in compact:
        opening = _extract_number(compact, default=90.0)
        return {"command": "/gripper", "args": [str(opening)]}

    if "关闭夹爪" in compact or "闭合夹爪" in compact or "夹爪关闭" in compact:
        opening = _extract_number(compact, default=0.0)
        return {"command": "/gripper", "args": [str(opening)]}

    if any(token in compact for token in ("姿势", "pose", "摆个", "动一动", "运动", "move")):
        return {"command": "/pose", "args": []}

    return None


def _extract_color(text: str) -> str | None:
    color_map = {
        "red": ("red", "红", "红色"),
        "blue": ("blue", "蓝", "蓝色"),
        "yellow": ("yellow", "黄", "黄色"),
    }
    for color, tokens in color_map.items():
        if any(token in text for token in tokens):
            return color
    return None


def _extract_number(text: str, *, default: float) -> float:
    digits = []
    started = False
    for char in text:
        if char.isdigit() or (char == "." and started):
            digits.append(char)
            started = True
        elif started:
            break
    try:
        return float("".join(digits)) if digits else float(default)
    except ValueError:
        return float(default)


def _summarize_detect_blocks(result: dict[str, Any]) -> str:
    if not result.get("ok"):
        return f"助手 > 视觉检测失败：{result.get('message', 'unknown error')}"
    detections = result.get("detections") or []
    items = [item for item in detections if isinstance(item, dict) and item.get("color")]
    if not items:
        return "助手 > 当前没有检测到颜色块。"
    target = result.get("target") or {}
    target_color = target.get("color") or result.get("target_color") or items[0].get("color")
    lines = [f"助手 > 当前看到 {len(items)} 个颜色块，优先目标是 {target_color}："]
    for item in items:
        lines.append(
            "  - "
            f"{item.get('color')}: "
            f"x={_format_number(item.get('x'))}, "
            f"y={_format_number(item.get('y'))}, "
            f"z={_format_number(item.get('z'))}"
        )
    return "\n".join(lines)


def _summarize_pick_color(result: dict[str, Any]) -> str:
    if result.get("cancelled"):
        return "助手 > 已取消抓取。"
    target = result.get("target") or {}
    color = target.get("color") or "目标"
    if result.get("ok"):
        return f"助手 > 抓取 {color} 的流程已完成。"
    failed = result.get("failed_step")
    message = result.get("message") or (f"失败步骤：{failed}" if failed else "unknown error")
    return f"助手 > 抓取 {color} 失败：{message}"


def _summarize_gripper(result: dict[str, Any]) -> str:
    if result.get("cancelled"):
        return "助手 > 已取消夹爪动作。"
    if result.get("ok"):
        reached = result.get("reached_opening_mm")
        if reached is not None:
            return f"助手 > 夹爪命令已发送，到达开度约 {float(reached):.1f} mm。"
        return "助手 > 夹爪命令已发送。"
    return f"助手 > 夹爪动作失败：{result.get('message', 'unknown error')}"


def _summarize_pose(result: dict[str, Any]) -> str:
    if result.get("cancelled"):
        return "助手 > 已取消摆姿势。"
    if result.get("ok"):
        final = result.get("final_pose", {}).get("position", {})
        return f"助手 > 姿势已摆好，位置: x={_format_number(final.get('x'))} y={_format_number(final.get('y'))} z={_format_number(final.get('z'))}"
    return f"助手 > 摆姿势失败：{result.get('message', 'unknown error')}"


def _generate_random_pose() -> dict[str, Any]:
    import random
    x = round(random.uniform(0.15, 0.35), 2)
    y = round(random.uniform(-0.1, 0.1), 2)
    z = round(random.uniform(0.25, 0.4), 2)
    return {
        "x": x,
        "y": y,
        "z": z,
        "roll_deg": 0.0,
        "pitch_deg": 0.0,
        "yaw_deg": round(random.uniform(-30, 30), 1),
        "duration": 2.0,
    }


async def _call_mcp_tool_with_retry(mcp, tool_name: str, arguments: dict, max_retries: int = 5) -> dict[str, Any]:
    result = await _call_mcp_tool(mcp, tool_name, arguments)
    if result.get("ok"):
        return result
    for attempt in range(max_retries - 1):
        arguments = _generate_random_pose()
        print(f"助手 > 重试中... 尝试 {attempt + 2}/{max_retries}")
        result = await _call_mcp_tool(mcp, tool_name, arguments)
        if result.get("ok"):
            return result
    return result


def _format_number(value: Any) -> str:
    try:
        return f"{float(value):.3f}"
    except (TypeError, ValueError):
        return "--"


async def _call_mcp_tool(mcp: Client, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
    try:
        result = await mcp.call_tool(name, arguments)
    except Exception as exc:
        return {"ok": False, "tool": name, "message": str(exc), "error_type": exc.__class__.__name__}
    payload = _mcp_result_to_json(result)
    return payload if isinstance(payload, dict) else {"ok": True, "tool": name, "result": payload}


def _confirm_tool(name: str, arguments: dict[str, Any]) -> bool:
    print(f"\n即将执行运动工具: {name} {_compact_json(arguments, limit=260)}")
    answer = input("确认执行？输入 y 继续，其它取消 > ").strip().lower()
    return answer in {"y", "yes", "是", "确认"}


def _mcp_tool_to_chat_tool(tool: Any) -> dict[str, Any]:
    name = str(getattr(tool, "name", ""))
    description = str(getattr(tool, "description", "") or f"MCP tool {name}")
    schema = (
        getattr(tool, "inputSchema", None)
        or getattr(tool, "input_schema", None)
        or getattr(tool, "parameters", None)
        or {"type": "object", "properties": {}}
    )
    schema = _plain_json(schema)
    if not isinstance(schema, dict):
        schema = {"type": "object", "properties": {}}
    schema.setdefault("type", "object")
    schema.setdefault("properties", {})
    return {
        "type": "function",
        "function": {
            "name": name,
            "description": description,
            "parameters": schema,
        },
    }


_TOOL_PARAMS = {
    "move_to_pose": {"x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg", "duration"},
    "move_joints": {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "duration"},
    "set_gripper_opening_mm": {"opening_mm"},
    "pick_color": {"color"},
    "detect_blocks": {"preferred_color"},
    "get_robot_status": set(),
    "safe_home": set(),
}

def _parse_tool_arguments(value: Any, tool_name: str = "") -> dict[str, Any]:
    if isinstance(value, dict):
        parsed = value
    elif not value:
        return {}
    else:
        try:
            parsed = json.loads(str(value))
        except json.JSONDecodeError:
            return {}
    if not isinstance(parsed, dict):
        return {}
    allowed_params = _TOOL_PARAMS.get(tool_name, set())
    if allowed_params:
        return {k: v for k, v in parsed.items() if k in allowed_params}
    return parsed


def _mcp_result_to_json(result: Any) -> Any:
    structured = getattr(result, "structured_content", None)
    if structured is not None:
        return _plain_json(structured)
    data = getattr(result, "data", None)
    if data is not None:
        return _plain_json(data)

    payload: dict[str, Any] = {}
    content = getattr(result, "content", None)
    if content is not None:
        payload["content"] = [
            _plain_json(getattr(item, "text", item)) for item in list(content)
        ]
    if hasattr(result, "is_error"):
        payload["is_error"] = bool(getattr(result, "is_error"))
    return payload


def _plain_json(value: Any) -> Any:
    if hasattr(value, "model_dump"):
        value = value.model_dump()
    try:
        return json.loads(json.dumps(value, ensure_ascii=False, default=str))
    except TypeError:
        return str(value)


def _compact_json(value: Any, *, limit: int) -> str:
    text = json.dumps(_plain_json(value), ensure_ascii=False, separators=(",", ":"), default=str)
    if len(text) <= limit:
        return text
    return text[: max(int(limit) - 20, 20)] + "...<truncated>"


def _parse_tool_calls_from_text(text: str) -> list[dict[str, Any]]:
    import re
    calls = []
    text = text.strip()
    tool_pattern = r"(move_to_pose|move_joints|set_gripper_opening_mm|pick_color|detect_blocks|get_robot_status|safe_home)\s+([^;]+)"
    matches = re.finditer(tool_pattern, text, re.IGNORECASE)
    for match in matches:
        name = match.group(1).lower()
        args_text = match.group(2)
        arguments = {}
        kv_pattern = r"(\w+)\s*=\s*([\d.]+)"
        kv_matches = re.finditer(kv_pattern, args_text)
        for kv in kv_matches:
            key = kv.group(1)
            try:
                value = float(kv.group(2))
            except ValueError:
                value = kv.group(2)
            arguments[key] = value
        if arguments:
            calls.append({
                "function": {
                    "name": name,
                    "arguments": json.dumps(arguments),
                },
                "id": f"tool-{int(time.time() * 1000)}-{len(calls)}",
            })
    return calls


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a text LLM agent for reBotArm MCP tools.")
    parser.add_argument("--mcp-url", default=os.getenv("REBOTARM_MCP_URL", DEFAULT_MCP_URL))
    parser.add_argument(
        "--base-url",
        default=os.getenv("REBOTARM_LLM_BASE_URL", DEFAULT_BASE_URL),
        help="OpenAI-compatible base URL.",
    )
    parser.add_argument(
        "--api-key",
        default=(
            os.getenv("REBOTARM_LLM_API_KEY")
            or os.getenv("DASHSCOPE_API_KEY")
            or os.getenv("OPENAI_API_KEY")
            or ""
        ),
    )
    parser.add_argument(
        "--model",
        default=os.getenv("REBOTARM_LLM_MODEL", DEFAULT_MODEL),
        help="Chat Completions model name.",
    )
    parser.add_argument("--timeout-sec", type=float, default=float(os.getenv("REBOTARM_LLM_TIMEOUT_SEC", "60")))
    parser.add_argument("--temperature", type=float, default=float(os.getenv("REBOTARM_LLM_TEMPERATURE", "0.7")))
    parser.add_argument("--max-tool-rounds", type=int, default=8)
    parser.add_argument("--result-chars", type=int, default=6000)
    parser.add_argument(
        "--verbose-tools",
        action="store_true",
        help="Print raw JSON for local MCP shortcut commands.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Do not ask before motion tools. Use only in a safe simulation.",
    )
    parser.add_argument(
        "--http-server",
        action="store_true",
        help="Run as an HTTP server for web UI integration (POST /chat).",
    )
    parser.add_argument(
        "--http-host",
        default=os.getenv("REBOTARM_AGENT_HTTP_HOST", "0.0.0.0"),
        help="HTTP server bind host.",
    )
    parser.add_argument(
        "--http-port",
        type=int,
        default=int(os.getenv("REBOTARM_AGENT_HTTP_PORT", "8082")),
        help="HTTP server bind port.",
    )
    args, _ros_args = parser.parse_known_args(argv)
    return args


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    has_api_key = bool(args.api_key)
    is_local = "localhost" in args.base_url or "127.0.0.1" in args.base_url
    # HTTP server mode: start even without API key (Dashboard/tools work, /chat returns error)
    if getattr(args, "http_server", False):
        if not has_api_key and not is_local:
            print(
                "[text-agent-http] WARNING: No API key set. /chat endpoint will return errors. "
                "Set REBOTARM_LLM_API_KEY, DASHSCOPE_API_KEY, or OPENAI_API_KEY to enable LLM chat. "
                "Dashboard, /tools and /call_tool work without an API key.",
                file=sys.stderr,
            )
        return asyncio.run(run_http_server(args))
    # REPL mode: API key required
    if not has_api_key and not is_local:
        print(
            "Missing API key. Set REBOTARM_LLM_API_KEY, DASHSCOPE_API_KEY, or OPENAI_API_KEY, "
            "or point --base-url to a local OpenAI-compatible server.",
            file=sys.stderr,
        )
        return 2
    try:
        return asyncio.run(run_repl(args))
    except KeyboardInterrupt:
        print()
        return 130



# ============ MCP Dashboard ============

_MCP_DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>reBotArm MCP Dashboard</title>
<style>
:root{--bg:#111211;--surface:#191b1a;--surface-2:#202321;--line:rgba(255,255,255,.12);--text:#f4f1ea;--muted:#a7ada7;--teal:#33d6b0;--amber:#f2a541;--red:#ef5a4d;--green:#77c96b;--blue:#5fa8ff;--purple:#a78bfa;--pink:#e879f9}
*{box-sizing:border-box}
body{margin:0;min-height:100vh;background:var(--bg);color:var(--text);font-family:Inter,"Segoe UI","Microsoft YaHei",Arial,sans-serif;display:flex;flex-direction:column}
.header{display:flex;align-items:center;justify-content:space-between;padding:16px 24px;border-bottom:1px solid var(--line);background:var(--surface)}
.header h1{margin:0;font-size:20px}
.header .status{display:flex;align-items:center;gap:8px;font-size:13px;color:var(--muted)}
.dot{width:8px;height:8px;border-radius:50%;background:var(--red);transition:.3s}
.dot.online{background:var(--green)}
.main{flex:1;display:grid;grid-template-columns:1fr 380px;gap:0;overflow:hidden}
.left{padding:20px;overflow-y:auto}
.right{border-left:1px solid var(--line);background:var(--surface);display:flex;flex-direction:column;overflow:hidden}
.cat-section{margin-bottom:24px}
.cat-title{display:flex;align-items:center;gap:8px;font-size:14px;font-weight:700;margin-bottom:12px;text-transform:uppercase;letter-spacing:.5px}
.cat-badge{width:10px;height:10px;border-radius:3px}
.tools-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));gap:12px}
.tool-card{background:var(--surface-2);border:1px solid var(--line);border-radius:8px;padding:14px;transition:.2s}
.tool-card:hover{border-color:rgba(255,255,255,.2)}
.tool-name{font-size:14px;font-weight:600;margin-bottom:4px}
.tool-desc{font-size:12px;color:var(--muted);margin-bottom:10px;line-height:1.4}
.tool-params{display:flex;flex-direction:column;gap:6px;margin-bottom:10px}
.param-row{display:flex;align-items:center;gap:8px}
.param-row label{font-size:11px;color:var(--muted);min-width:90px;font-family:monospace}
.param-row input{flex:1;background:var(--bg);border:1px solid var(--line);border-radius:4px;padding:5px 8px;color:var(--text);font-size:12px;min-width:0}
.param-row input:focus{outline:none;border-color:var(--teal)}
.btn-call{background:var(--teal);color:#111211;border:none;border-radius:4px;padding:6px 14px;font-size:12px;font-weight:600;cursor:pointer;transition:.2s}
.btn-call:hover{opacity:.85}
.btn-call:disabled{opacity:.4;cursor:not-allowed}
.motion-tag{display:inline-block;font-size:10px;background:rgba(242,165,65,.15);color:var(--amber);padding:1px 6px;border-radius:3px;margin-left:6px}
.chat-section{flex:1;display:flex;flex-direction:column;padding:16px;overflow:hidden}
.chat-log{flex:1;overflow-y:auto;margin-bottom:12px;font-size:13px;line-height:1.5}
.chat-log .msg{margin-bottom:8px;padding:8px 10px;border-radius:6px}
.chat-log .msg.user{background:var(--surface-2)}
.chat-log .msg.assistant{background:rgba(51,214,176,.08);border-left:2px solid var(--teal)}
.chat-log .msg.tool{background:rgba(95,168,255,.08);border-left:2px solid var(--blue);font-family:monospace;font-size:11px}
.chat-log .msg.error{background:rgba(239,90,77,.08);border-left:2px solid var(--red)}
.chat-input-row{display:flex;gap:8px}
.chat-input-row input{flex:1;background:var(--bg);border:1px solid var(--line);border-radius:6px;padding:8px 12px;color:var(--text);font-size:13px;min-width:0}
.chat-input-row input:focus{outline:none;border-color:var(--teal)}
.chat-input-row button{background:var(--teal);color:#111211;border:none;border-radius:6px;padding:8px 16px;font-size:13px;font-weight:600;cursor:pointer;white-space:nowrap}
.chat-input-row button:disabled{opacity:.4;cursor:not-allowed}
.loading{display:flex;align-items:center;justify-content:center;height:100%;color:var(--muted);font-size:14px}
</style>
</head>
<body>
<div class="header">
  <h1>reBotArm MCP Dashboard</h1>
  <div class="status"><span class="dot" id="dot"></span><span id="status-text">未连接</span></div>
</div>
<div class="main">
  <div class="left" id="tools-container">
    <div class="loading">正在加载工具列表...</div>
  </div>
  <div class="right">
    <div class="chat-section">
      <div style="font-size:14px;font-weight:700;margin-bottom:12px;color:var(--teal)">自然语言控制</div>
      <div class="chat-log" id="chat-log"></div>
      <div class="chat-input-row">
        <input type="text" id="chat-input" placeholder="输入指令，如：回到零位、打开夹爪、抓红色方块" disabled/>
        <button id="chat-btn" disabled>发送</button>
      </div>
    </div>
  </div>
</div>
<script>
const MOTION_TOOLS = ["safe_home","gravity_compensation_start","set_gripper_opening_mm","move_to_pose","move_joints","pick_color","record_replay"];

async function loadTools(){
  try{
    const r = await fetch("/tools");
    const data = await r.json();
    if(!data.ok){throw new Error(data.error||"加载失败")}
    renderTools(data.tools);
    document.getElementById("dot").classList.add("online");
    document.getElementById("status-text").textContent = `${"connected"} ${data.tools.length} tools`;
    document.getElementById("chat-input").disabled = false;
    document.getElementById("chat-btn").disabled = false;
  }catch(e){
    document.getElementById("tools-container").innerHTML = `<div class="loading" style="color:var(--red)">加载失败: ${e.message}<br><button onclick="loadTools()" style="margin-top:8px;background:var(--teal);border:none;border-radius:4px;padding:4px 12px;cursor:pointer">重试</button></div>`;
    document.getElementById("status-text").textContent = "连接失败";
  }
}

function renderTools(tools){
  const cats = {};
  tools.forEach(t=>{
    const info = t.category || ["其他","#a7ada7"];
    const catName = info[0];
    if(!cats[catName]) cats[catName] = {color:info[1], tools:[]};
    cats[catName].tools.push(t);
  });
  let html = "";
  for(const [name, info] of Object.entries(cats)){
    html += `<div class="cat-section"><div class="cat-title"><span class="cat-badge" style="background:${info.color}"></span>${name}</div><div class="tools-grid">`;
    for(const t of info.tools){
      const isMotion = MOTION_TOOLS.includes(t.name);
      html += `<div class="tool-card"><div class="tool-name">${t.name}${isMotion?'<span class="motion-tag">运动</span>':""}</div><div class="tool-desc">${t.description||""}</div>`;
      const params = t.parameters?.properties || {};
      const required = t.parameters?.required || [];
      if(Object.keys(params).length > 0){
        html += '<div class="tool-params">';
        for(const [pname, pinfo] of Object.entries(params)){
          const req = required.includes(pname);
          const def = pinfo.default !== undefined ? pinfo.default : "";
          const ptype = pinfo.type || "string";
          html += `<div class="param-row"><label>${pname}${req?"*":""}</label><input type="${ptype==="number"?"number":"text"}" data-tool="${t.name}" data-param="${pname}" value="${def}" placeholder="${ptype}"/></div>`;
        }
        html += '</div>';
      }
      html += `<button class="btn-call" onclick="callTool('${t.name}')">调用</button></div>`;
    }
    html += '</div></div>';
  }
  document.getElementById("tools-container").innerHTML = html;
}

async function callTool(name){
  const inputs = document.querySelectorAll(`input[data-tool="${name}"]`);
  const args = {};
  inputs.forEach(inp=>{
    const val = inp.value.trim();
    if(val === "") return;
    const ptype = inp.placeholder;
    if(ptype === "number" || ptype === "integer"){
      args[inp.dataset.param] = parseFloat(val);
    } else if(ptype === "boolean"){
      args[inp.dataset.param] = val === "true";
    } else {
      args[inp.dataset.param] = val;
    }
  });
  const btn = event.target;
  btn.disabled = true;
  btn.textContent = "执行中...";
  addLog("tool", `调用 ${name}(${JSON.stringify(args)})`);
  try{
    const r = await fetch("/call_tool", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({name, arguments:args})});
    const data = await r.json();
    addLog("tool", `${name} 结果: ${JSON.stringify(data).slice(0,500)}`);
  }catch(e){
    addLog("error", `调用失败: ${e.message}`);
  }
  btn.disabled = false;
  btn.textContent = "调用";
}

function addLog(type, text){
  const log = document.getElementById("chat-log");
  const div = document.createElement("div");
  div.className = "msg " + type;
  div.textContent = text;
  log.appendChild(div);
  log.scrollTop = log.scrollHeight;
}

async function sendChat(){
  const input = document.getElementById("chat-input");
  const btn = document.getElementById("chat-btn");
  const text = input.value.trim();
  if(!text) return;
  addLog("user", text);
  input.value = "";
  btn.disabled = true;
  btn.textContent = "等待...";
  try{
    const r = await fetch("/chat", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({text})});
    const data = await r.json();
    if(data.ok){
      addLog("assistant", data.text || "(无回复)");
      if(data.events){
        for(const ev of data.events){
          if(ev.type === "tool"){
            addLog("tool", `${ev.name}(${JSON.stringify(ev.arguments)}) → ${JSON.stringify(ev.result).slice(0,300)}`);
          } else if(ev.type === "error"){
            addLog("error", ev.message);
          }
        }
      }
    } else {
      addLog("error", data.error || "请求失败");
    }
  }catch(e){
    addLog("error", e.message);
  }
  btn.disabled = false;
  btn.textContent = "发送";
}

document.getElementById("chat-btn").addEventListener("click", sendChat);
document.getElementById("chat-input").addEventListener("keydown", e=>{if(e.key==="Enter") sendChat()});
loadTools();
</script>
</body>
</html>"""


async def _http_list_tools(args: argparse.Namespace) -> dict[str, Any]:
    """List MCP tools with categories for the dashboard."""
    try:
        async with Client(args.mcp_url) as mcp:
            mcp_tools = await mcp.list_tools()
            tools = []
            for tool in mcp_tools:
                name = str(getattr(tool, "name", ""))
                cat_info = TOOL_CATEGORIES.get(name, ("其他", "#a7ada7"))
                chat_tool = _mcp_tool_to_chat_tool(tool)
                tools.append({
                    "name": name,
                    "description": str(getattr(tool, "description", "") or ""),
                    "parameters": chat_tool["function"]["parameters"],
                    "category": cat_info,
                    "is_motion": name in MOTION_TOOLS,
                })
            return {"ok": True, "tools": tools}
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


async def _http_call_tool(args: argparse.Namespace, payload: dict[str, Any]) -> dict[str, Any]:
    """Call a single MCP tool directly."""
    name = str(payload.get("name", "")).strip()
    arguments = payload.get("arguments") or {}
    if not name:
        return {"ok": False, "error": "missing tool name"}
    try:
        async with Client(args.mcp_url) as mcp:
            result = await _call_mcp_tool(mcp, name, arguments)
            return result
    except Exception as exc:
        return {"ok": False, "error": str(exc)}


# ============ HTTP Server mode ============

# Per-request state holder for the HTTP server
_HTTP_STATE: dict[str, Any] = {}


async def _http_handle_chat(args: argparse.Namespace, payload: dict[str, Any]) -> dict[str, Any]:
    """Process a single chat turn over HTTP."""
    if not args.api_key and "localhost" not in args.base_url and "127.0.0.1" not in args.base_url:
        return {
            "ok": False,
            "error": "LLM API key not configured. Set REBOTARM_LLM_API_KEY, DASHSCOPE_API_KEY, or OPENAI_API_KEY. Tool listing and direct tool calls still work.",
        }
    user_text = str(payload.get("text") or payload.get("message") or "").strip()
    reset = bool(payload.get("reset", False))
    if not user_text:
        return {"ok": False, "error": "empty text"}

    if reset or "messages" not in _HTTP_STATE:
        _HTTP_STATE["messages"] = [{"role": "system", "content": SYSTEM_PROMPT}]

    events: list[dict[str, Any]] = []
    final_text = ""

    async with Client(args.mcp_url) as mcp:
        intent = _parse_builtin_intent(user_text)
        if intent is not None:
            command = intent["command"]
            parts = [command, *intent.get("args", [])]
            if command == "/pose":
                arguments = _generate_random_pose()
                result = await _call_mcp_tool_with_retry(mcp, "move_to_pose", arguments, max_retries=5)
                events.append({"type": "assistant", "content": "正在摆姿势..."})
                events.append({
                    "type": "tool",
                    "name": "move_to_pose",
                    "arguments": arguments,
                    "result": result,
                })
                if result.get("ok"):
                    final = result.get("final_pose", {}).get("position", {})
                    final_text = f"姿势已摆好，位置: x={_format_number(final.get('x'))} y={_format_number(final.get('y'))} z={_format_number(final.get('z'))}"
                else:
                    final_text = f"摆姿势失败：{result.get('message', 'unknown error')}"
                events.append({"type": "assistant", "content": final_text})
                return {"ok": True, "text": final_text, "events": events}
            elif command == "/detect":
                color = parts[1] if len(parts) > 1 else "auto"
                result = await _call_mcp_tool(mcp, "detect_blocks", {"preferred_color": color})
                events.append({"type": "assistant", "content": _summarize_detect_blocks(result)})
                return {"ok": True, "text": _summarize_detect_blocks(result), "events": events}
            elif command == "/pick":
                color = parts[1] if len(parts) > 1 else "auto"
                result = await _call_mcp_tool(mcp, "pick_color", {"color": color})
                events.append({"type": "assistant", "content": _summarize_pick_color(result)})
                return {"ok": True, "text": _summarize_pick_color(result), "events": events}
            elif command == "/gripper":
                if len(parts) >= 2:
                    try:
                        opening_mm = float(parts[1])
                        result = await _call_mcp_tool(mcp, "set_gripper_opening_mm", {"opening_mm": opening_mm})
                        events.append({"type": "assistant", "content": _summarize_gripper(result)})
                        return {"ok": True, "text": _summarize_gripper(result), "events": events}
                    except ValueError:
                        pass

        llm = ChatCompletionsLLM(
            base_url=args.base_url,
            api_key=args.api_key,
            model=args.model,
            timeout_sec=args.timeout_sec,
            temperature=args.temperature,
        )

        mcp_tools = await mcp.list_tools()
        tools = [_mcp_tool_to_chat_tool(tool) for tool in mcp_tools]

        _HTTP_STATE["messages"].append({"role": "user", "content": user_text})

        for _ in range(max(1, int(args.max_tool_rounds))):
            try:
                response = llm.complete(_HTTP_STATE["messages"], tools)
            except Exception as exc:
                events.append({"type": "error", "message": str(exc)})
                return {"ok": False, "error": str(exc), "events": events}

            choice = (response.get("choices") or [{}])[0]
            message = choice.get("message") or {}
            content = message.get("content") or ""
            tool_calls = message.get("tool_calls") or []

            if content:
                events.append({"type": "assistant", "content": content})
                final_text = content

            if not tool_calls:
                if content:
                    _HTTP_STATE["messages"].append({"role": "assistant", "content": content})
                break

            assistant_message = {"role": "assistant", "content": content, "tool_calls": tool_calls}
            _HTTP_STATE["messages"].append(assistant_message)

            for call in tool_calls:
                function = call.get("function") or {}
                name = str(function.get("name") or "")
                arguments = _parse_tool_arguments(function.get("arguments"), name)
                tool_call_id = str(call.get("id") or f"tool-{int(time.time() * 1000)}")

                if not name:
                    tool_result = {"ok": False, "message": "LLM emitted a tool call without a name."}
                elif args.yes is False and name in MOTION_TOOLS:
                    # In HTTP mode, always run motion tools (auto-yes by default)
                    tool_result = await _call_mcp_tool(mcp, name, arguments)
                else:
                    tool_result = await _call_mcp_tool(mcp, name, arguments)

                events.append({
                    "type": "tool",
                    "name": name,
                    "arguments": arguments,
                    "result": tool_result,
                })

                _HTTP_STATE["messages"].append({
                    "role": "tool",
                    "tool_call_id": tool_call_id,
                    "content": _compact_json(tool_result, limit=args.result_chars),
                })
        else:
            events.append({"type": "info", "message": "工具调用轮次已到上限。"})

    return {"ok": True, "text": final_text, "events": events}


class _ChatHTTPHandler(BaseHTTPRequestHandler):
    server_args: argparse.Namespace = None  # set in run_http_server

    def log_message(self, format, *args):  # noqa: A002
        sys.stderr.write("[text-agent-http] " + (format % args) + "\n")

    def _write_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):  # noqa: N802
        self._write_json(204, {})

    def _write_html(self, status: int, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):  # noqa: N802
        if self.path == "/" or self.path == "/dashboard":
            self._write_html(200, _MCP_DASHBOARD_HTML.encode("utf-8"))
        elif self.path == "/health":
            self._write_json(200, {"ok": True, "service": "rebotarm-text-agent"})
        elif self.path == "/tools":
            result = asyncio.run(_http_list_tools(self.server_args))
            self._write_json(200, result)
        else:
            self._write_json(404, {"ok": False, "error": "not found"})

    def do_POST(self):  # noqa: N802
        if self.path not in ("/chat", "/call_tool"):
            self._write_json(404, {"ok": False, "error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length > 0 else b"{}"
            payload = json.loads(raw.decode("utf-8"))
        except Exception as exc:
            self._write_json(400, {"ok": False, "error": f"invalid json: {exc}"})
            return

        try:
            if self.path == "/chat":
                result = asyncio.run(_http_handle_chat(self.server_args, payload))
            else:
                result = asyncio.run(_http_call_tool(self.server_args, payload))
        except Exception as exc:
            self._write_json(500, {"ok": False, "error": str(exc)})
            return

        self._write_json(200, result)


async def run_http_server(args: argparse.Namespace) -> int:
    host = str(getattr(args, "http_host", "0.0.0.0"))
    port = int(getattr(args, "http_port", 8082))

    _ChatHTTPHandler.server_args = args
    httpd = ThreadingHTTPServer((host, port), _ChatHTTPHandler)
    print(f"[text-agent-http] listening on http://{host}:{port}/ (dashboard) /chat (llm) /tools (list) /call_tool (invoke)", flush=True)
    print(f"[text-agent-http] MCP={args.mcp_url} model={args.model}", flush=True)

    loop = asyncio.get_event_loop()

    def _serve():
        httpd.serve_forever()

    serve_task = loop.run_in_executor(None, _serve)
    try:
        await asyncio.Event().wait()  # run forever
    except KeyboardInterrupt:
        pass
    finally:
        httpd.shutdown()
        serve_task.cancel()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
