"""CLI for ``rosclaw eurdf`` — manifest-driven e-URDF-Zoo commands."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def add_eurdf_subparser(
    subparsers: argparse._SubParsersAction[argparse.ArgumentParser],
) -> argparse.ArgumentParser:
    """Register the ``rosclaw eurdf`` subcommand tree."""
    eurdf_parser = subparsers.add_parser("eurdf", help="e-URDF-Zoo manifest asset commands")
    eurdf_subparsers = eurdf_parser.add_subparsers(dest="eurdf_command")
    eurdf_parser.set_defaults(_parser=eurdf_parser)

    # pull
    pull_parser = eurdf_subparsers.add_parser(
        "pull", help="Pull an asset bundle into the local cache"
    )
    pull_parser.add_argument(
        "asset_id", help="Asset identifier (e.g., dexhands/inspire_hand/right)"
    )
    pull_parser.add_argument("--version", default="latest", help="Asset version")
    pull_parser.add_argument("--source", type=Path, default=None, help="Explicit source zoo path")
    pull_parser.add_argument(
        "--zoo-path", type=Path, default=None, help="e-URDF-Zoo robots directory"
    )

    # info
    info_parser = eurdf_subparsers.add_parser("info", help="Show detailed asset information")
    info_parser.add_argument("asset_id", help="Asset identifier")
    info_parser.add_argument("--version", default="latest", help="Asset version")
    info_parser.add_argument("--source", type=Path, default=None, help="Explicit source zoo path")
    info_parser.add_argument(
        "--zoo-path", type=Path, default=None, help="e-URDF-Zoo robots directory"
    )
    info_parser.add_argument("--json", action="store_true", help="Output JSON")

    # search
    search_parser = eurdf_subparsers.add_parser("search", help="Search available assets")
    search_parser.add_argument("query", help="Search query")
    search_parser.add_argument(
        "--zoo-path", type=Path, default=None, help="e-URDF-Zoo robots directory"
    )
    search_parser.add_argument("--json", action="store_true", help="Output JSON")

    # validate
    validate_parser = eurdf_subparsers.add_parser("validate", help="Validate an asset bundle")
    validate_parser.add_argument("asset_id", help="Asset identifier")
    validate_parser.add_argument("--version", default="latest", help="Asset version")
    validate_parser.add_argument(
        "--source", type=Path, default=None, help="Explicit source zoo path"
    )
    validate_parser.add_argument(
        "--zoo-path", type=Path, default=None, help="e-URDF-Zoo robots directory"
    )
    validate_parser.add_argument("--json", action="store_true", help="Output JSON")

    # cache
    cache_parser = eurdf_subparsers.add_parser("cache", help="Manage the local asset cache")
    cache_subparsers = cache_parser.add_subparsers(dest="cache_command")
    cache_list_parser = cache_subparsers.add_parser("list", help="List cached assets")
    cache_list_parser.add_argument("--json", action="store_true", help="Output JSON")

    return eurdf_parser


def _client(zoo_path: Path | None = None):
    """Create a EurdfZooClient, handling missing dependency gracefully."""
    try:
        from rosclaw.eurdf.zoo_client import EurdfZooClient
    except Exception as exc:
        print(f"[ROSClaw] e-URDF-Zoo client unavailable: {exc}", file=sys.stderr)
        sys.exit(1)
    return EurdfZooClient(zoo_path=zoo_path)


def dispatch_eurdf_command(args: argparse.Namespace) -> int:
    """Route ``rosclaw eurdf`` subcommands."""
    command = getattr(args, "eurdf_command", None)
    if command is None:
        # Print help for the eurdf subcommand tree.
        parser = getattr(args, "_parser", None)
        if parser is not None:
            parser.print_help()
        return 1

    if command == "pull":
        return cmd_eurdf_pull(args)
    if command == "info":
        return cmd_eurdf_info(args)
    if command == "search":
        return cmd_eurdf_search(args)
    if command == "validate":
        return cmd_eurdf_validate(args)
    if command == "cache":
        cache_command = getattr(args, "cache_command", None)
        if cache_command == "list":
            return cmd_eurdf_cache_list(args)
        print("[ROSClaw] eurdf cache: expected 'list'")
        return 1

    print(f"[ROSClaw] Unknown eurdf command: {command}")
    return 1


def cmd_eurdf_pull(args: argparse.Namespace) -> int:
    """Pull an asset into the local cache."""
    client = _client(zoo_path=args.zoo_path)
    try:
        cached_path = client.pull(args.asset_id, version=args.version, source_path=args.source)
    except Exception as exc:
        print(f"[ROSClaw] eurdf pull failed: {exc}")
        return 1

    print(f"[ROSClaw] Pulled {args.asset_id}@{args.version}")
    print(f"  cache: {cached_path}")
    print(f"\nNext:\n  rosclaw body init --robot {args.asset_id}")
    return 0


def cmd_eurdf_info(args: argparse.Namespace) -> int:
    """Show detailed information about a manifest asset."""
    client = _client(zoo_path=args.zoo_path)
    try:
        asset = client.load(args.asset_id, version=args.version, source_path=args.source)
    except Exception as exc:
        print(f"[ROSClaw] eurdf info failed: {exc}")
        return 1

    manifest = asset.manifest
    info = {
        "id": asset.asset_id,
        "name": asset.name,
        "category": asset.category,
        "type": asset.robot_type,
        "dof": asset.dof,
        "status": asset.status,
        "version": asset.version,
        "path": str(asset.base_path),
        "is_manifest": asset.is_manifest,
    }
    if manifest is not None:
        info["vendor"] = manifest.asset.vendor
        info["model"] = manifest.asset.model
        info["variant"] = manifest.asset.variant
        info["description"] = manifest.asset.description
        info["real_robot_execution_allowed"] = manifest.runtime_policy.real_robot_execution_allowed
        info["sandbox_required"] = manifest.runtime_policy.sandbox_required
        if asset.capabilities:
            info["capabilities"] = [
                {"name": cap.name, "risk": cap.risk} for cap in asset.capabilities.capabilities
            ]
            info["forbidden_capabilities"] = [
                {"id": fc.id, "description": fc.description}
                for fc in asset.capabilities.forbidden_capabilities
            ]

    if args.json:
        print(json.dumps(info, indent=2, default=str))
        return 0

    print("=" * 60)
    print("e-URDF-Zoo Asset Info")
    print("=" * 60)
    for key, value in info.items():
        if key in {"capabilities", "forbidden_capabilities"}:
            print(f"\n{key}:")
            for item in value:
                print(f"  - {item}")
        else:
            print(f"  {key}: {value}")
    print("=" * 60)
    return 0


def cmd_eurdf_search(args: argparse.Namespace) -> int:
    """Search the zoo for assets matching a query."""
    client = _client(zoo_path=args.zoo_path)
    try:
        results = client.search_assets(args.query)
    except Exception as exc:
        print(f"[ROSClaw] eurdf search failed: {exc}")
        return 1

    payload = [
        {
            "id": r.id,
            "name": r.name,
            "category": r.category,
            "status": r.status,
            "version": r.version,
            "is_legacy": r.is_legacy,
            "path": str(r.path),
        }
        for r in results
    ]

    if args.json:
        print(json.dumps(payload, indent=2, default=str))
        return 0

    print("=" * 60)
    print(f"e-URDF-Zoo Search: '{args.query}'")
    print("=" * 60)
    if not payload:
        print("No matching assets found.")
    else:
        print(f"{'ID':<40} {'Name':<25} {'Category':<15}")
        print("-" * 80)
        for item in payload:
            print(f"{item['id']:<40} {item['name']:<25} {item['category']:<15}")
    print("=" * 60)
    return 0


def cmd_eurdf_validate(args: argparse.Namespace) -> int:
    """Validate a manifest asset bundle."""
    client = _client(zoo_path=args.zoo_path)
    try:
        report = client.validate(args.asset_id, version=args.version, source_path=args.source)
    except Exception as exc:
        print(f"[ROSClaw] eurdf validate failed: {exc}")
        return 1

    if args.json:
        print(json.dumps(report, indent=2, default=str))
        return 0 if report.get("overall") != "FAIL" else 1

    print("=" * 60)
    print(f"e-URDF-Zoo Validation: {report.get('overall')}")
    print("=" * 60)
    print(f"Asset: {report.get('asset_id')}")
    for filename, result in report.get("results", {}).items():
        status = result.get("status")
        print(f"\n  {filename}: {status}")
        for msg in result.get("messages", []):
            print(f"    [{msg.get('level')}] {msg.get('message')}")
    print("=" * 60)
    return 0 if report.get("overall") != "FAIL" else 1


def cmd_eurdf_cache_list(args: argparse.Namespace) -> int:
    """List cached assets."""
    client = _client()
    results = client.cache_list()

    if args.json:
        print(json.dumps(results, indent=2, default=str))
        return 0

    print("=" * 60)
    print("e-URDF-Zoo Cache")
    print("=" * 60)
    if not results:
        print("No cached assets.")
    else:
        print(f"{'Asset ID':<40} {'Version':<10} {'Path':<30}")
        print("-" * 80)
        for item in results:
            print(f"{item['asset_id']:<40} {item['version']:<10} {item['path']:<30}")
    print("=" * 60)
    return 0
