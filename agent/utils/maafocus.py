"""向 MXU 发送 focus 消息，向其他客户端输出可见日志。"""

from maa.context import Context
from utils import pienv
from utils.logger import logger
from utils.i18n import T

_FOCUS_NODE = "_MAANTE_FOCUS_"


def Print(context: Context, content: str):
    """通过客户端支持的通道显示文本；不做 i18n。"""
    if pienv.client_name().strip().upper() != "MXU" or context is None:
        logger.info("%s", content)
        return

    pipeline_override = {
        _FOCUS_NODE: {
            "focus": {"Node.Action.Starting": content},
            "action": "DoNothing",
            "pre_delay": 0,
            "post_delay": 0,
        }
    }

    try:
        result = context.run_action(_FOCUS_NODE, pipeline_override=pipeline_override)
    except Exception as e:
        logger.debug(
            "Failed to send focus: %s",
            e,
            exc_info=True,
            extra={"file_only": True},
        )
        logger.info("%s", content)
        return

    if result is None or not result.success:
        logger.info("%s", content)


def PrintT(context: Context, key: str, *args):
    """通过可见通道显示翻译文本；*args 为 %-格式化参数。"""
    Print(context, T(key, *args))
