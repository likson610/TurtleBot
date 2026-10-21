import os
import re
import json
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WiFiScannerNode(Node):
    def __init__(self):
        super().__init__('wifi_scanner_node')
        
        self.interface = os.environ.get('WIFI_INTERFACE', 'wlan1')
        self.freqs = os.environ.get('SCAN_FREQUENCIES', '').strip()
        
        self.publisher_ = self.create_publisher(String, '/wifi/scan_raw', 10)
        
        timer_period = 1.0 if self.freqs else 3.0
        self.timer = self.create_timer(timer_period, self.scan_and_publish)
    
        print(f"[START] Węzeł wifi_scanner uruchomiony na interfejsie {self.interface}", flush=True)

    def scan_and_publish(self):
        cmd = ["iw", "dev", self.interface, "scan"]
        if self.freqs:
            cmd.extend(["freq"] + self.freqs.split())

        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            networks = self._parse_iw_scan(result.stdout)
            
            payload = {
                "timestamp": self.get_clock().now().nanoseconds / 1e9,
                "frame_id": "base_link",
                "interface": self.interface,
                "count": len(networks),
                "networks": networks
            }
            
            msg = String()
            msg.data = json.dumps(payload)
            self.publisher_.publish(msg)
            
            print(f"[SCAN] Opublikowano {len(networks)} sieci na /wifi/scan_raw", flush=True)
            
        except subprocess.CalledProcessError as e:
            if "Device or resource busy" in e.stderr:
                print("[WARN] Interfejs zajęty innym skanem, ponawianie za chwilę...", flush=True)
            else:
                print(f"[ERROR iw]: {e.stderr.strip()}", flush=True)
        except Exception as ex:
            print(f"[ERROR]: {ex}", flush=True)

    def _parse_iw_scan(self, raw_output):
        networks = []
        current_ap = None

        for line in raw_output.splitlines():
            line = line.strip()
            bss_match = re.match(r'^BSS\s+([0-9a-fA-F:]{17})', line)
            if bss_match:
                if current_ap:
                    networks.append(current_ap)
                current_ap = {
                    "bssid": bss_match.group(1).lower(),
                    "ssid": "",
                    "rssi_dbm": None,
                    "freq_mhz": None
                }
                continue

            if not current_ap:
                continue

            freq_match = re.match(r'^freq:\s+(\d+)', line)
            if freq_match:
                current_ap["freq_mhz"] = int(freq_match.group(1))

            signal_match = re.match(r'^signal:\s+([-\d.]+)\s*dBm', line)
            if signal_match:
                current_ap["rssi_dbm"] = float(signal_match.group(1))

            ssid_match = re.match(r'^SSID:\s*(.*)', line)
            if ssid_match:
                current_ap["ssid"] = ssid_match.group(1)

        if current_ap:
            networks.append(current_ap)

        return networks

def main(args=None):
    rclpy.init(args=args)
    node = WiFiScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"[CRITICAL]: {e}", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()