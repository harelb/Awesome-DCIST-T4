"""tx_rx_monitor notes parsing, radio-interface default and /api/network/interfaces."""

from adt4_console.netstats import default_radio, parse_tx_rx_notes

OLD = "[|IF|TX|RX|Δt|]:|enp118s0|284.74|33255.83|1.00|::|tailscale0|1.00|2.00|0.50|:"
NEW = (
    "[|IF|TX|RX|Δt|]:|enp118s0|2.00|1.00|1.00|::|wlan0|ERR|ERR|ERR|:"
    " ;; TOT[|IF|TXTOT_B|RXTOT_B|SINCE_S|ADDR|]"
    ":|enp118s0|123456|654321|60.0|192.168.100.2|::|wlan0|ERR|ERR|ERR||:"
)
TOPO = {
    "networks": {
        "mit_wifi": {"subnet": "10.29.0.0/16"},
        "silvus": {"subnet": "192.168.100.0/24"},
    },
    "machines": {
        "r1": {"role": "robot", "addresses": {"silvus": "192.168.100.2",
                                              "mit_wifi": "10.29.1.5"}},
    },
}  # fmt: skip


def test_parse_old_format_rates_only():
    ifs = parse_tx_rx_notes(OLD)
    assert set(ifs) == {"enp118s0", "tailscale0"}
    assert ifs["enp118s0"]["tx_rate_Bps"] == 284.74 * 1024
    assert ifs["tailscale0"]["rx_rate_Bps"] == 2.0 * 1024 / 0.5
    assert ifs["enp118s0"]["tx_total_B"] is None and not ifs["enp118s0"]["error"]


def test_parse_new_format_with_totals_and_errors():
    ifs = parse_tx_rx_notes(NEW)
    e = ifs["enp118s0"]
    assert e["tx_rate_Bps"] == 2048.0 and e["rx_rate_Bps"] == 1024.0
    assert (e["tx_total_B"], e["rx_total_B"], e["since_s"]) == (123456, 654321, 60.0)
    assert e["addr"] == "192.168.100.2" and not e["error"]
    assert ifs["wlan0"]["error"] and ifs["wlan0"]["tx_total_B"] is None
    assert parse_tx_rx_notes("") == {} and parse_tx_rx_notes("garbage") == {}


def test_default_radio():
    ifs = {
        "eth0": {"addr": "192.168.100.2"},
        "wlan0": {"addr": "10.29.1.5"},
        "tailscale0": {"addr": "100.64.0.3"},
    }
    assert default_radio(ifs, TOPO, "silvus", "r1") == ("eth0", "silvus")
    assert default_radio(ifs, TOPO, "mit_wifi", "r1") == ("wlan0", "mit_wifi")
    # the address the console reaches the machine on wins
    assert default_radio(ifs, TOPO, "silvus", "r1", ip="10.29.1.5") == (
        "wlan0", "mit_wifi")  # fmt: skip
    # unknown machine: subnet match on the active network
    assert default_radio({"x": {"addr": "10.29.9.9"}}, TOPO, "mit_wifi", "zz") == (
        "x", "mit_wifi")  # fmt: skip
    assert default_radio({"x": {"addr": "1.2.3.4"}}, TOPO, "mit_wifi", "zz") == (
        None, None)  # fmt: skip


def test_interfaces_endpoint(client):
    svc = client.app.state.services
    svc.fleet.update_nodes(
        {
            "hamilton": [
                {"nickname": "tx_rx_monitor", "status": 1, "notes": NEW.replace(
                    "192.168.100.2", "10.31.144.106")},
                {"nickname": "hydra", "status": 1, "notes": ""},
            ],
            "gauss": [{"nickname": "hydra", "status": 1, "notes": ""}],
        }
    )
    r = client.get("/api/network/interfaces").json()
    assert r["network"] == "mit_wifi"
    assert set(r["machines"]) == {"hamilton"}
    h = r["machines"]["hamilton"]
    assert h["has_totals"] and h["radio_default"] == "enp118s0"
    assert h["interfaces"]["enp118s0"]["rx_total_B"] == 654321
