#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

import sys
import socket
import json

def set_keepalive(sock, after_idle_sec=1, interval_sec=3, max_fails=5):
	"""Set TCP keepalive on an open socket.

	It activates after 1 second (after_idle_sec) of idleness,
	then sends a keepalive ping once every 3 seconds (interval_sec),
	and closes the connection after 5 failed ping (max_fails), or 15 seconds
	"""
	sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
	sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, after_idle_sec)
	sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, interval_sec)
	sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, max_fails)
if not hasattr(socket, "set_keepalive"):
	socket.set_keepalive = set_keepalive

class JsonClient:
	def __init__(self, addr = ("arm", 10002)):
		self.pSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.pSocket.settimeout(1)
		socket.set_keepalive(self.pSocket)
		self.pSocket.connect(addr)
		self.lMsgs = []

	def write(self, cmd):
		data = {'command': cmd}
		num = self.pSocket.sendall(json.dumps(data))
		while True:
			msg = json.loads(self.pSocket.recv(4096))
			if msg.has_key("return"): return msg["return"]
			elif msg.has_key("error"): return msg["error"]
			self.lMsgs.insert(0, msg)

	def read(self):
		if len(self.lMsgs) > 0:
			return self.lMsgs.pop()
		self.pSocket.setblocking(False)
		try:
			return json.loads(self.pSocket.recv(4096))
		except socket.error:
			pass
		finally:
			self.pSocket.setblocking(True)

if __name__ == "__main__":
	print JsonClient().write(sys.argv[1])

	#from datetime import datetime
	#from time import sleep
	#pClient = JsonClient()
	#while True:
	#	msg = pClient.read()
	#	if msg:
	#		print "Got async", msg
	#	print datetime.now(), float(pClient.write("get distance forward lower"))
	#	sleep(0.1)
