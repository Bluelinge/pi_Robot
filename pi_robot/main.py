from __future__ import annotations

import uvicorn

from pi_robot.app import app, settings


def main() -> None:
    uvicorn.run(app, host=settings.host, port=settings.port)


if __name__ == "__main__":
    main()
