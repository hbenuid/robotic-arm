"""The motor dashboard: a local web page for driving and testing one MKS board (`uv run dashboard`).

bus (the one thread on the CAN bus) -> bench (the board under test: units, limits, safety) -> routines (the tests) ->
server (the HTTP server and the page in static/). It sits on top of the package's layers like the CLIs, and imports
nothing outside the standard library, python-can and pyserial.
"""
