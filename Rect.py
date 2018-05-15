# -*- coding: utf-8 -*-
"""
Rect - helps to deal with screen space coordinates.

A rect is defined by the two points (minx, miny) and (maxx, maxy). 
On a computer screen with a coordinate system starting in the left
upper corner minx, miny, maxx, maxy represent left, top, right and
bottom. When dealing with integer pixel coordinates [minx, maxx) 
and [miny, maxy) are considered to be half-closed intervals, the  
point (maxx, maxy) lies outside the rect.
"""

import math


class Rect:
  def __init__(self, minX, minY, maxX, maxY):
    self.minX = minX
    self.minY = minY
    self.maxX = maxX
    self.maxY = maxY

  def empty(self):
    return Rect(0, 0, 0, 0)

  def fromXYWidthHeight(self, x, y, width, height):
    return Rect(x, y, x + width, y + height)

  def fromCenterWidthHeight(self, centerX, centerY, width, height):
    return Rect.fromXYWidthHeight(self, centerX - width * 0.5, centerY - height * 0.5, width, height)

  def scale(self, factorX, factorY):
    if factorY == None :
      factorY = factorX
    return Rect(self.minX * factorX, self.minY * factorY, self.maxX * factorX, self.maxY * factorY)
  

  def inflate(self, x, y):
    return Rect(self.minX - x, self.minY - y, self.maxX + x, self.maxY + y)
  

  def size(self):
    return self.width(), self.height()
  

  def width(self):
    return self.maxX-self.minX
  

  def height(self):
    return self.maxY-self.minY
  

  def area(self):
    return self.width() * self.height()
  

  def center(self):
    return (self.minX + self.maxX) / 2, (self.minY + self.maxY) / 2
  

  def isEmpty(self):
    return self.minX == self.maxX and self.minY == self.maxY
  

  def clip(self, clipRect):
    return Rect(
      min(max(self.minX, clipRect.minX), clipRect.maxX),
      min(max(self.minY, clipRect.minY), clipRect.maxY),
      max(min(self.maxX, clipRect.maxX), clipRect.minX),
      max(min(self.maxY, clipRect.maxY), clipRect.minY)
    )
  

  def containsPt(self, x, y):
    return self.minX <= x and x < self.maxX and self.minY <= y and y < self.maxY
  

  def contains(self, otherRect):
    return self.containsPt(otherRect.minX, otherRect.minY) and self.containsPt(otherRect.maxX, otherRect.maxY)
  

  def overlaps(self, other):
    return self.minX < other.maxX and self.maxX > other.minX and self.minY < other.maxY and self.maxY > other.minY
  

  def normalize(self):
    l = 0
    t = 0
    r = 0
    b = 0
    if self.minX <= self.maxX :
      l = self.minX
      r = self.maxX
    else :
      l = self.maxX
      r = self.minX
    
    if self.minY <= self.maxY :
      t = self.minY
      b = self.maxY
    else :
      t = self.maxY
      b = self.minY
    return Rect(l, t, r, b)
  

  def unpack(self):
    return self.minX, self.minY, self.maxX, self.maxY
  
  
  def union(self, b):
    minx = min(self.minX, b.minX)
    miny = min(self.minY, b.minY)
    maxx = max(self.maxX, b.maxX)
    maxy = max(self.maxY, b.maxY)
    return Rect(minx, miny, maxx, maxy)
  

  def intersect(self, b):
    minx = max(self.minX, b.minX)
    miny = max(self.minY, b.minY)
    maxx = min(self.maxX, b.maxX)
    maxy = min(self.maxY, b.maxY)
    if maxx >= minx and maxy >= miny :
      return Rect(minx, miny, maxx, maxy)
    else :
      return Rect(0, 0, 0, 0)
    
  
  def IoU(self, b):
    i = self.intersect(b).area() 
    return i / (self.area() + b.area() - i)
  

  def toArray(self):
    return [self.minX, self.minY, self.maxX, self.maxY]
  

  def snapToInt(self):
    return Rect(math.floor(self.minX), math.floor(self.minY), math.ceil(self.maxX), math.ceil(self.maxY))
  

  def offset(self, x, y):
    return Rect(self.minX + x, self.minY + y, self.maxX + x, self.maxY + y)
  

  # returns vertices in clockwise order
  def vertices(self):
    return [ (self.minX, self.minY),
             (self.maxX, self.minY),
             (self.maxX, self.maxY),
             (self.minX, self.maxY) ]


  def clone(self):
    return Rect(self.minX, self.minY, self.maxX, self.maxY)
  

  def tostring(self):
    return "min: ({:.2f}, {:.2f}), max: ({:.2f}, {:.2f}), size: ({:.2f} x {:.2f})".format(self.minX, self.minY, self.maxX, self.maxY, self.width(), self.height())
