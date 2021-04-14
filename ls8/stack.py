from collections import deque

class Stack:
  def __init__(self):
    self.stack = deque()
    
  def push(self, value):
    self.stack.append(value)
    
  def pop(self):
    return self.stack.pop()
  
  def size(self):
    return len(self.stack)