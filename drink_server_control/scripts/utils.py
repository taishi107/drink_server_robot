# coding: utf-8
# author: Fumihiro Ueki
# date] 2019.09.26

import numpy as np
import subprocess

def calc_gravity_point(x_min, y_min, x_max, y_max):
    '''
    バウンディングボックスの座標から
    物体の重心位置を計算する

    input : 
    (x_min, y_min) : ボックスの左上の座標
    (x_max, y_max) : ボックスの右下の座標

    return :
    gx : x軸の重心位置
    gy : y軸の重心位置
    '''
    
    gx = (x_max + x_min) / 2
    gy = (y_max + y_min) / 2

    return gx, gy

def amount_of_movement(gx, gy, img_width=640, img_height=480):
   '''
   重心位置からロボットがどれだけ動かすかを計算する
   
   input : 
   gx : x軸の重心位置
   gy : y軸の重心位置

   return : 
   物体の重心位置が画像の中心位置からどれだけ
   ずれているか(単位はピクセル)
   '''

   center_x = img_width // 2
   center_y = img_height // 2

   diff_x = gx - center_x 
   diff_y = gy - center_y 

   return diff_x, diff_y

def calc_box_size(x_min, y_min, x_max, y_max):
    '''
    ボックスの座標から物体の大きさを計算
    input : 
    (x_min, y_min) : ボックスの左上の座標
    (x_max, y_max) : ボックスの右下の座標
    '''

    w = x_max - x_min
    h = y_max - y_min

    return w * h

def jtalk(text):
   open_jtalk=['open_jtalk']
   mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
   htsvoice=['-m','/usr/share/hts-voice/mei/mei_normal.htsvoice']
   speed=['-r','1.0']
   outwav=['-ow','open_jtalk.wav']
   cmd=open_jtalk+mech+htsvoice+speed+outwav
   c = subprocess.Popen(cmd,stdin=subprocess.PIPE)
   c.stdin.write(text)
   c.stdin.close()
   c.wait()
   aplay = ['aplay','-q','open_jtalk.wav']
   wr = subprocess.Popen(aplay)
