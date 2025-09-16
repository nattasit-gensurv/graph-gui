import json
with open("config/robots.json", "r") as f:
            data = json.load(f)
print(data)