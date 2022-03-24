n



def make_screen(ev3, run_name, page, check1, check2, check3, check4):
    ev3.screen.clear()
    ev3.screen.draw_text(1, 1, run_name)
    ev3.screen.draw_text(1, 20, page)
    ev3.screen.draw_text(1, 40, "> " + check1)
    ev3.screen.draw_text(1, 60, "> " + check2)
    ev3.screen.draw_text(1, 80, "> " + check3)
    ev3.screen.draw_text(1, 100, "> " + check4)



    