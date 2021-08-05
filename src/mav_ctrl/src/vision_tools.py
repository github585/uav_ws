#mode 0:down;mode 1;front mode;2:left;mode 3:right;mode 4;behind
def pixel2move(pixel_x=320,pixel_y=240,mode=0,distance=1,k=0.002):
    center_x=320
    center_y=240
    delta_pixel_x=pixel_x-center_x
    delta_pixel_y=pixel_y-center_y
    if mode==0:
        move_1=-delta_pixel_y*distance*k
        move_2=-delta_pixel_x*distance*k
        return [move_1,move_2,0]
    elif mode==1:
        move_3=-delta_pixel_y*distance*k
        move_2=-delta_pixel_x*distance*k
        return [0,move_2,move_3]
    elif mode==2:
        move_1=delta_pixel_x*distance*k
        move_3=-delta_pixel_y*distance*k
        return [move_1,0,move_3]
    elif mode==3:
        move_1=-delta_pixel_x*distance*k
        move_3=-delta_pixel_y*distance*k
        return [move_1,0,move_3]
    elif mode==4:
        move_3=-delta_pixel_y*distance*k
        move_2=delta_pixel_x*distance*k
        return[0,move_2,move_3]