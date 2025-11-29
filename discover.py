import time


def discover_scope_servers(stop_after=99, timeout=1):
    from zeroconf import ServiceBrowser, ServiceListener, Zeroconf

    addr = []

    class MyListener(ServiceListener):

        def update_service(self, zc: Zeroconf, type_: str, name: str) -> None:
            print(f"Service {name} updated")

        def remove_service(self, zc: Zeroconf, type_: str, name: str) -> None:
            print(f"Service {name} removed")

        def add_service(self, zc: Zeroconf, type_: str, name: str) -> None:
            info = zc.get_service_info(type_, name)
            addr.extend((a, info.port, info.server) for a in info.parsed_addresses())

    zeroconf = Zeroconf()
    listener = MyListener()
    # browser = ServiceBrowser(zeroconf, "_http._tcp.local.", listener)
    browser = ServiceBrowser(zeroconf, "_scope._tcp.local.", listener)

    t0 = time.time()

    addrs = set()

    try:
        while time.time() - t0 < timeout:
            if addr:
                addrs.update(addr)
                if len(addrs) > stop_after:
                    return addrs
            time.sleep(.1)
    finally:
        zeroconf.close()

    return list(addrs)
