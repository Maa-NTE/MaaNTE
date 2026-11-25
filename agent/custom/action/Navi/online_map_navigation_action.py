from typing import Any

from maa.agent.agent_server import AgentServer
from maa.context import Context
from maa.custom_action import CustomAction

from utils.maafocus import PrintT

from ..Common.logger import get_logger
from .navigation_server import NavigationPortUnavailableError
from .route_websocket_service import RouteWebSocketService
from .route_runner import RouteRunner
from .route_model import RouteSession

logger = get_logger(__name__)


@AgentServer.custom_action("online_map_navigation")
class OnlineMapNavigationAction(CustomAction):
    def run(
        self, context: Context, argv: CustomAction.RunArg
    ) -> CustomAction.RunResult:
        try:
            params = self.load_option_params(context)
            port = int(params.get("port", 14514))
            tolerance = float(params.get("tolerance", 5.0))
            frame_interval = max(0.05, float(params.get("frame_interval", 0.1)))
            angle_backend = str(params.get("angle_backend", "auto"))
            position_backend = str(params.get("position_backend", "auto"))
            debug = bool(params.get("debug", False))
        except ValueError as exc:
            logger.debug(
                "OnlineMapNavigation param invalid: %s",
                exc,
                exc_info=True,
                extra={"file_only": True},
            )
            PrintT(context, "navi.invalid_settings")
            return CustomAction.RunResult(success=False)

        route = RouteSession()
        runner = RouteRunner(
            context,
            route,
            angle_backend=angle_backend,
            position_backend=position_backend,
            tolerance=tolerance,
            frame_interval=frame_interval,
            debug=debug,
        )
        network = RouteWebSocketService(
            route,
            port=port,
            get_source_size=runner.source_size,
            get_current_point=runner.current_point,
        )
        runner.on_frame = network.publish_frame

        try:
            network.start()
            runner.start()
            logger.debug(
                "OnlineMapNavigation service started: ws://0.0.0.0:%s",
                port,
                extra={"file_only": True},
            )
            PrintT(context, "navi.started", port)
            runner.run_until_stopped(on_tick=network.publish_route)
            return CustomAction.RunResult(success=False)
        except NavigationPortUnavailableError as exc:
            PrintT(context, "navi.port_unavailable", exc.port)
            return CustomAction.RunResult(success=False)
        except Exception as exc:
            logger.debug(
                "OnlineMapNavigation failed: %s",
                exc,
                exc_info=True,
                extra={"file_only": True},
            )
            PrintT(context, "navi.failed")
            return CustomAction.RunResult(success=False)
        finally:
            runner.close()
            network.stop()

    @staticmethod
    def load_option_params(context: Context) -> dict[str, Any]:
        params: dict[str, Any] = {}

        settings = OnlineMapNavigationAction.load_config_attach(
            context, "OnlineMapNavigationSettingsConfig"
        )
        for key in ("port", "tolerance", "frame_interval"):
            if key in settings:
                params[key] = settings[key]

        position = OnlineMapNavigationAction.load_config_attach(
            context, "OnlineMapNavigationPositionBackendConfig"
        )
        if position.get("position_backend") in {"auto", "coordinate", "map"}:
            params["position_backend"] = position["position_backend"]

        angle = OnlineMapNavigationAction.load_config_attach(
            context, "OnlineMapNavigationAngleBackendConfig"
        )
        if angle.get("angle_backend") in {
            "auto",
            "directml",
            "cpu",
        }:
            params["angle_backend"] = angle["angle_backend"]

        debug = OnlineMapNavigationAction.load_config_attach(
            context, "OnlineMapNavigationDebugConfig"
        )
        if "debug" in debug:
            params["debug"] = debug["debug"]

        return params

    @staticmethod
    def load_config_attach(context: Context, node_name: str) -> dict[str, Any]:
        node_data = context.get_node_data(node_name) or {}
        attach = node_data.get("attach")
        return attach if isinstance(attach, dict) else {}
