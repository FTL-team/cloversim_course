#!/usr/bin/env python3

# Write your checker there

from .randomization import *
from cloversim.score import ScoreTask, Scoring
from http.server import BaseHTTPRequestHandler, HTTPServer

hello_task = ScoreTask('Hello', 5)
land_task = ScoreTask('Land color', 10)
qr_task = ScoreTask('QRcode contents', 10)

scoring = Scoring('Camera task', [hello_task, land_task, qr_task])
scoring.update()


def mark_task(task, success):
  if success:
    task.score = task.max_score
    task.failed = False
  else:
    task.score = 0
    task.failed = True
  task.update()


class RequestHandler(BaseHTTPRequestHandler):

  def do_POST(self):
    content_length = int(self.headers.get('Content-Length', 0))
    post_data = self.rfile.read(content_length)
    body = post_data.decode('utf-8').strip()
    if self.path == '/hello':
      mark_task(hello_task, body == 'hello')
    elif self.path == '/land':
      mark_task(land_task, body == land_color)
    elif self.path == '/qr':
      mark_task(qr_task, body == qrcode_contents)
    else:
      self.send_response(404)
      self.end_headers()
      self.wfile.write("Not found")
      return

    self.send_response(200)
    self.end_headers()


httpd = HTTPServer(('0.0.0.0', 8080), RequestHandler)
httpd.serve_forever()
