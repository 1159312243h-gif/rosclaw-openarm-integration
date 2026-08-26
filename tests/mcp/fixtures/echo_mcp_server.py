"""Minimal echo MCP server for testing the stdio protocol handshake.

This server is spawned by tests that want to exercise the real MCP
initialize handshake instead of mocking ``_handshake_stdio``.
"""

from __future__ import annotations

import asyncio

from mcp.server import Server
from mcp.server.stdio import stdio_server
from mcp.types import Tool


async def main() -> None:
    server = Server("echo")

    @server.list_tools()
    async def list_tools() -> list[Tool]:
        return [
            Tool(
                name="echo",
                description="Echo the provided message.",
                inputSchema={"type": "object", "properties": {}},
            ),
        ]

    async with stdio_server() as (read, write):
        await server.run(read, write, server.create_initialization_options())


if __name__ == "__main__":
    asyncio.run(main())
