#!/usr/bin/env python3

# Write your checker there

from .randomization import *
import json
import math
from cloversim.score import ScoreTask, Scoring
from http.server import BaseHTTPRequestHandler, HTTPServer

marker_tasks = {
    'red': ScoreTask('Red marker', 20),
    'green': ScoreTask('Green marker', 20),
    'blue': ScoreTask('Blue marker', 20)
}

scoring = Scoring('Position known task', list(marker_tasks.values()))
scoring.update()


def mark_score(color, score):
  if score < 0:
    marker_tasks[color].score = 0
    marker_tasks[color].failed = True
  else:
    marker_tasks[color].score = score
    marker_tasks[color].failed = False
  marker_tasks[color].update()


class RequestHandler(BaseHTTPRequestHandler):

  def do_POST(self):
    content_length = int(self.headers.get('Content-Length', 0))
    post_data = self.rfile.read(content_length)
    body = post_data.decode('utf-8').strip()
    try:
      body = json.loads(body)
      print(body)
      for color in marker_tasks:
        if color not in body:
          mark_score(color, -1)
          continue
        user_pos = body[color]
        actual_pos = MARKER_POSITIONS[color]
        dist = math.sqrt((actual_pos[0] - user_pos[0])**2 +
                         (actual_pos[1] - user_pos[1])**2) * 100
        print(user_pos, actual_pos, dist)
        mark_score(color, 20 - dist)

      self.send_response(200)
      self.end_headers()
    except Exception as e:
      print(e)
      self.send_response(400)
      self.end_headers()


httpd = HTTPServer(('0.0.0.0', 8080), RequestHandler)
httpd.serve_forever()
