# Copyright 2024 darshan - Apache-2.0
"""
dds_probe.py - DDS Domain Conflict Detection

Scans UDP ports for DDS domain conflicts and network issues.
DDS domain port formula: 7400 + 250 * domain_id

Detects:
  - Multiple DDS implementations on same domain
  - Port conflicts with other applications
  - Domain ID mismatches between nodes
  - Multicast configuration issues
"""

import os
import socket
import struct
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .finding import Finding

try:
    import psutil
    _HAS_PSUTIL = True
except ImportError:  # pragma: no cover - optional dependency
    psutil = None
    _HAS_PSUTIL = False


@dataclass
class DDSDomainInfo:
    """Information about a DDS domain."""
    domain_id: int
    discovery_port: int  # Participant discovery multicast port
    user_port_base: int  # Base port for user traffic
    is_active: bool = False
    processes: List[str] = None
    
    def __post_init__(self):
        if self.processes is None:
            self.processes = []


# DDS port calculation constants
# See: https://www.omg.org/spec/DDSI-RTPS/2.5/PDF
# PB = Port Base = 7400
# DG = Domain Gain = 250
# PG = Participant Gain = 2
# d0 = Discovery Multicast Port Offset = 0
# d1 = Discovery Unicast Port Offset = 10
# d2 = User Multicast Port Offset = 1
# d3 = User Unicast Port Offset = 11
DDS_PORT_BASE = 7400
DDS_DOMAIN_GAIN = 250
DDS_PARTICIPANT_GAIN = 2

# Standard DDS implementations
DDS_IMPLEMENTATIONS = {
    'fastdds': 'Fast DDS (eProsima)',
    'cyclonedds': 'Cyclone DDS (Eclipse)',
    'connext': 'Connext DDS (RTI)',
    'opendds': 'OpenDDS',
}


def calculate_dds_ports(domain_id: int, participant_id: int = 0) -> Dict[str, int]:
    """
    Calculate DDS ports for a given domain ID.
    
    Returns dict with:
      - discovery_multicast: Multicast port for participant discovery
      - discovery_unicast: Unicast port for discovery
      - user_multicast: Multicast port for user data
      - user_unicast: Unicast port for user data
    """
    d0 = 0   # Discovery multicast offset
    d1 = 10  # Discovery unicast offset
    d2 = 1   # User multicast offset
    d3 = 11  # User unicast offset
    
    pb = DDS_PORT_BASE
    dg = DDS_DOMAIN_GAIN
    pg = DDS_PARTICIPANT_GAIN
    d = domain_id
    p = participant_id
    
    return {
        'discovery_multicast': pb + dg * d + d0,
        'discovery_unicast': pb + dg * d + d1 + pg * p,
        'user_multicast': pb + dg * d + d2,
        'user_unicast': pb + dg * d + d3 + pg * p,
    }


def get_ros_domain_id() -> int:
    """Get the current ROS_DOMAIN_ID from environment."""
    try:
        return int(os.environ.get('ROS_DOMAIN_ID', '0'))
    except (ValueError, TypeError):
        return 0


def check_port_in_use(port: int, protocol: str = 'udp') -> bool:
    """
    Check if a port is in use.
    
    Args:
        port: Port number to check
        protocol: 'udp' or 'tcp'
        
    Returns:
        True if port is in use
    """
    sock_type = socket.SOCK_DGRAM if protocol == 'udp' else socket.SOCK_STREAM
    
    try:
        with socket.socket(socket.AF_INET, sock_type) as sock:
            sock.settimeout(0.1)
            result = sock.connect_ex(('127.0.0.1', port))
            # For UDP, we try to bind instead
            if protocol == 'udp':
                try:
                    sock.bind(('', port))
                    return False
                except OSError:
                    return True
            return result == 0
    except Exception:
        return False


def get_port_processes(port: int) -> List[str]:
    """Get list of processes using a specific port."""
    # Policy: avoid subprocess calls. Use psutil if available; otherwise
    # return an empty list (we still report other DDS findings).
    if not _HAS_PSUTIL:
        return []

    processes: set[str] = set()
    try:
        for kind in ('udp', 'tcp'):
            try:
                conns = psutil.net_connections(kind=kind)
            except Exception:
                conns = []
            for conn in conns:
                laddr = conn.laddr
                pid = conn.pid
                if pid is None:
                    continue
                port_num: int | None = None
                if hasattr(laddr, "port"):
                    port_num = laddr.port  # type: ignore[assignment]
                elif isinstance(laddr, tuple) and len(laddr) >= 2:
                    maybe_port = laddr[1]
                    if isinstance(maybe_port, int):
                        port_num = maybe_port
                if port_num != port:
                    continue
                try:
                    processes.add(psutil.Process(pid).name())
                except Exception:
                    # AccessDenied or process already exited: ignore.
                    continue
    except Exception:
        return []

    return list(processes)


def probe_domain(domain_id: int) -> DDSDomainInfo:
    """
    Probe a DDS domain for activity.
    
    Args:
        domain_id: DDS domain ID to probe
        
    Returns:
        DDSDomainInfo with probing results
    """
    ports = calculate_dds_ports(domain_id)
    discovery_port = ports['discovery_multicast']
    user_port = ports['user_multicast']
    
    # Check if ports are in use
    is_active = check_port_in_use(discovery_port) or check_port_in_use(user_port)
    
    # Get processes if active
    processes = []
    if is_active:
        processes.extend(get_port_processes(discovery_port))
        processes.extend(get_port_processes(user_port))
    
    return DDSDomainInfo(
        domain_id=domain_id,
        discovery_port=discovery_port,
        user_port_base=user_port,
        is_active=is_active,
        processes=list(set(processes))
    )


def scan_domains(
    domain_range: range = None,
    max_domains: int | None = None,
) -> List[DDSDomainInfo]:
    """
    Scan multiple DDS domains for activity.

    Args:
        domain_range: Range of domain IDs to scan (default: 0-10).
                      Mutually exclusive with max_domains.
        max_domains:  Convenience shorthand — scan the first N domain IDs
                      (0 to max_domains-1) and always include the current
                      ROS_DOMAIN_ID.

    Returns:
        List of active DDSDomainInfo objects.
    """
    if domain_range is None:
        if max_domains is not None:
            # Build a range that covers 0..max_domains-1 AND the current domain.
            current = get_ros_domain_id()
            end = max(max_domains, current + 1)
            domain_range = range(0, end)
        else:
            domain_range = range(0, 11)

    active_domains = []

    for domain_id in domain_range:
        info = probe_domain(domain_id)
        if info.is_active:
            active_domains.append(info)

    return active_domains



def check_multicast_enabled() -> Tuple[bool, str]:
    """
    Check if multicast is enabled on the system.
    
    Returns:
        Tuple of (is_enabled, details)
    """
    # Policy: avoid subprocess calls. Best-effort test by joining a multicast
    # group. This doesn't guarantee DDS discovery will succeed, but it
    # provides a reasonable indicator without shelling out.
    group = "224.0.0.1"
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        try:
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, struct.pack("b", 1))
            mreq = struct.pack("4s4s", socket.inet_aton(group), socket.inet_aton("0.0.0.0"))
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            return True, "Multicast group join succeeded"
        finally:
            sock.close()
    except Exception as e:
        return False, f"Multicast group join failed: {e}"


def get_dds_implementation() -> Optional[str]:
    """Detect which DDS implementation is being used."""
    rmw = os.environ.get('RMW_IMPLEMENTATION', '')
    
    if 'fastrtps' in rmw or 'fastdds' in rmw:
        return 'fastdds'
    elif 'cyclonedds' in rmw:
        return 'cyclonedds'
    elif 'connext' in rmw:
        return 'connext'
    elif 'opendds' in rmw:
        return 'opendds'
    
    # Default for ROS 2 Humble+
    return 'fastdds'


def check_dds_domain(ignore_set: set = None) -> List[Finding]:
    """
    Check for DDS domain conflicts and configuration issues.
    
    Args:
        ignore_set: Set of checks to ignore
        
    Returns:
        List of findings for DDS issues
    """
    ignore_set = ignore_set or set()
    findings = []
    
    current_domain = get_ros_domain_id()
    dds_impl = get_dds_implementation()
    dds_name = DDS_IMPLEMENTATIONS.get(dds_impl, dds_impl or 'Unknown')
    
    # Check current domain ports
    current_ports = calculate_dds_ports(current_domain)
    
    # Probe current domain
    current_info = probe_domain(current_domain)
    
    if not current_info.is_active:
        findings.append(Finding(
            check='dds_domain',
            topic=f'domain_{current_domain}',
            severity=2,
            message=(
                f'DDS domain {current_domain} appears inactive. '
                f'Discovery port {current_ports["discovery_multicast"]} not in use.'
            ),
            suggestion=(
                'No ROS 2 nodes detected on this domain. '
                'Check: ros2 node list, ROS_DOMAIN_ID environment variable.'
            ),
            extra={
                'domain_id': current_domain,
                'discovery_port': current_ports['discovery_multicast'],
                'dds_implementation': dds_impl,
            }
        ))
    
    # Scan for other active domains (potential conflicts)
    scan_range = range(max(0, current_domain - 2), min(233, current_domain + 3))
    active_domains = scan_domains(scan_range)
    
    other_active = [d for d in active_domains if d.domain_id != current_domain]
    
    if other_active:
        # Find unexpected domain activity
        for domain_info in other_active:
            findings.append(Finding(
                check='dds_domain',
                topic=f'domain_{domain_info.domain_id}',
                severity=1,
                message=(
                    f'Other DDS domain {domain_info.domain_id} is active '
                    f'(port {domain_info.discovery_port}). '
                    f'Processes: {", ".join(domain_info.processes) or "unknown"}'
                ),
                suggestion=(
                    f'Nodes on domain {domain_info.domain_id} will not communicate '
                    f'with your domain {current_domain}. '
                    'Ensure ROS_DOMAIN_ID is consistent across all nodes.'
                ),
                extra={
                    'domain_id': domain_info.domain_id,
                    'current_domain': current_domain,
                }
            ))
    
    # Check multicast
    multicast_ok, multicast_detail = check_multicast_enabled()
    
    if not multicast_ok:
        findings.append(Finding(
            check='dds_domain',
            topic='multicast',
            severity=2,
            message=f'Multicast may not be configured: {multicast_detail}',
            suggestion=(
                'DDS discovery requires multicast. '
                'Check: ip route show, firewall rules (ufw status), '
                'and network interface configuration.'
            )
        ))
    
    # Check for loopback-only configuration
    localhost_only = os.environ.get('ROS_LOCALHOST_ONLY', '0')
    if localhost_only == '1':
        findings.append(Finding(
            check='dds_domain',
            topic='localhost_only',
            severity=1,
            message='ROS_LOCALHOST_ONLY=1 is set - network discovery disabled.',
            suggestion=(
                'Only localhost communication is enabled. '
                'Unset ROS_LOCALHOST_ONLY for multi-machine communication.'
            )
        ))
    
    # Add domain info summary if no issues
    if not findings:
        findings.append(Finding(
            check='dds_domain',
            topic=f'domain_{current_domain}',
            severity=1,
            message=(
                f'DDS domain {current_domain} active using {dds_name}. '
                f'Discovery port: {current_ports["discovery_multicast"]}.'
            ),
            suggestion='DDS configuration looks healthy.',
            extra={
                'domain_id': current_domain,
                'dds_implementation': dds_impl,
                'ports': current_ports,
            }
        ))
    
    return findings


def get_dds_summary() -> dict:
    """
    Get summary of DDS configuration.
    
    Returns:
        Dict with DDS configuration summary
    """
    current_domain = get_ros_domain_id()
    ports = calculate_dds_ports(current_domain)
    dds_impl = get_dds_implementation()
    multicast_ok, _ = check_multicast_enabled()
    localhost_only = os.environ.get('ROS_LOCALHOST_ONLY', '0') == '1'
    
    return {
        'domain_id': current_domain,
        'dds_implementation': dds_impl,
        'discovery_port': ports['discovery_multicast'],
        'user_port': ports['user_multicast'],
        'multicast_enabled': multicast_ok,
        'localhost_only': localhost_only,
        'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION', 'default'),
    }
