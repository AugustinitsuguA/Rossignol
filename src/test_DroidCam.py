import requests

URL = "http://192.168.0.20:4747/video"
r = requests.get(URL, timeout=5)
print(r.status_code, r.headers.get("Content-Type"))
print(r.text[:1000])