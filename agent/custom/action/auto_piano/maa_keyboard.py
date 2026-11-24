from __future__ import annotations

import ctypes
import time
from typing import TYPE_CHECKING

from .key_mapping import NOTE_KEY_MAPPING
from utils.logger import logger

if TYPE_CHECKING:
    from maa.controller import Controller

user32 = ctypes.windll.user32
WM_KEYDOWN = 0x0100
WM_KEYUP = 0x0101
WM_ACTIVATE = 0x0006
WA_CLICKACTIVE = 2

# 使用游戏专用的左 Shift 和左 Ctrl 防跑调。
WIN32_VK = {
    "shift": 0xA0,
    "ctrl": 0xA2,
    "enter": 0x0D,
    "0": 0x30,
    "1": 0x31,
    "2": 0x32,
    "3": 0x33,
    "4": 0x34,
    "5": 0x35,
    "6": 0x36,
    "7": 0x37,
    "8": 0x38,
    "9": 0x39,
    ";": 0xBA,
    "=": 0xBB,
    "-": 0xBD,
    "[": 0xDB,
    "]": 0xDD,
    "'": 0xDE,
    "a": 0x41,
    "b": 0x42,
    "c": 0x43,
    "d": 0x44,
    "e": 0x45,
    "f": 0x46,
    "g": 0x47,
    "h": 0x48,
    "i": 0x49,
    "j": 0x4A,
    "k": 0x4B,
    "l": 0x4C,
    "m": 0x4D,
    "n": 0x4E,
    "o": 0x4F,
    "p": 0x50,
    "q": 0x51,
    "r": 0x52,
    "s": 0x53,
    "t": 0x54,
    "u": 0x55,
    "v": 0x56,
    "w": 0x57,
    "x": 0x58,
    "y": 0x59,
    "z": 0x5A,
}

WINDOW_TITLES = [
    "NTE  ",
    "异环  ",
]


def get_lparam(vk_code, is_down=True):
    """构建底层硬件扫描码。"""
    scan_code = user32.MapVirtualKeyW(vk_code, 0)
    lparam = 1 | (scan_code << 16)
    if not is_down:
        lparam |= 0xC0000000
    return lparam


class MaaKeyboardBridge:
    def __init__(
        self,
        hold_seconds: float = 0.008,
        mapping: dict | None = None,
        controller: Controller | None = None,
    ):
        self.mapping = mapping if mapping is not None else NOTE_KEY_MAPPING
        self.hold_seconds = hold_seconds
        self.controller = controller
        self.hwnd = 0

        if self.controller is not None:
            if not self.controller.connected:
                raise RuntimeError("Maa keyboard controller is not connected.")
            logger.info("自动演奏使用当前 Maa 控制器发送按键。")
            return

        for title in WINDOW_TITLES:
            hwnd = user32.FindWindowW(None, title)
            if hwnd:
                self.hwnd = hwnd
                logger.info("已连接到游戏窗口: '%s' (HWND: %s)", title, self.hwnd)
                break

        if not self.hwnd:
            raise RuntimeError("Game window not found. Check that the game is running.")

    def _force_send_key(self, vk_code, is_down):
        """优先使用选定控制器；钢琴保留原有的 PostMessage 输入。"""
        if self.controller is not None:
            if is_down:
                job = self.controller.post_key_down(vk_code)
            else:
                job = self.controller.post_key_up(vk_code)
            if not job.wait().succeeded:
                raise RuntimeError(
                    "Maa keyboard %s failed (VK 0x%02X)."
                    % ("key_down" if is_down else "key_up", vk_code)
                )
            return

        lparam = get_lparam(vk_code, is_down)
        msg = WM_KEYDOWN if is_down else WM_KEYUP
        if not user32.PostMessageW(self.hwnd, msg, vk_code, lparam):
            raise ctypes.WinError()

    def _activate(self):
        if self.controller is None:
            user32.SendMessageW(self.hwnd, WM_ACTIVATE, WA_CLICKACTIVE, 0)

    def execute_chord(self, midi_notes):
        """按修饰键类型隔离并短按一个和弦。"""
        self._activate()
        self._execute_chord_groups(midi_notes)

    def _execute_chord_groups(self, midi_notes):
        """发送已经激活窗口的短按和弦。"""

        normal_keys, shift_keys, ctrl_keys = [], [], []

        for note in midi_notes:
            if note not in self.mapping:
                continue
            action = self.mapping[note]
            key = action.split("+")[-1]
            if "shift+" in action:
                shift_keys.append(key)
            elif "ctrl+" in action:
                ctrl_keys.append(key)
            else:
                normal_keys.append(key)

        self._press_group(normal_keys)
        self._press_group(shift_keys, "shift")
        self._press_group(ctrl_keys, "ctrl")

    def _press_group(self, keys, modifier: str | None = None):
        if not keys:
            return

        modifier_vk = WIN32_VK.get(modifier)
        vk_codes = list(dict.fromkeys(WIN32_VK[key] for key in keys if key in WIN32_VK))
        pressed = []
        try:
            if modifier_vk is not None:
                self._force_send_key(modifier_vk, True)
                pressed.append(modifier_vk)
                time.sleep(0.002)

            for vk in vk_codes:
                self._force_send_key(vk, True)
                pressed.append(vk)

            if self.hold_seconds > 0:
                time.sleep(self.hold_seconds)
        finally:
            release_error = None
            for vk in reversed(pressed):
                try:
                    if vk == modifier_vk:
                        time.sleep(0.001)
                    self._force_send_key(vk, False)
                except Exception as exc:
                    logger.error("Failed to release playback key 0x%02X: %s", vk, exc)
                    if release_error is None:
                        release_error = exc
            if release_error is not None:
                raise release_error
