package game

import rl "vendor:raylib"
//import fmt "core:fmt"

draw_powerbar :: proc() {
    screen_width := rl.GetScreenWidth()
    //screen_height := rl.GetScreenHeight()

    //fmt.println(screen_width, screen_height)

    scale :i32= 2
    off :i32= 25 * scale
    off2 :i32= 20 * scale
    size :f32= f32(10 * scale)
    color := rl.RED
    //center Circle

    
    step :f32 = 100.0
    start :f32 = 0.0
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2,  off2, size *0.75, color)

    //4
    color = fling_color(1)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 + off, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 - off, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 + off*2, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 - off*2, off2, size, color)
    //6
    color = fling_color(2)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 + off*3, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 - off*3, off2, size, color)
    //8
    color = fling_color(3)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 + off*4, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 - off*4, off2, size, color)
    //12
    color = fling_color(4)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 + off*5, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 - off*5, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 + off*6, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 - off*6, off2, size, color)
    //20
    color = fling_color(5)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 + off*7, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 - off*7, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 + off*8, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 - off*8, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 + off*9, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 - off*9, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 + off*10, off2, size, color)
    if g.fuel < start {color.a = 0} 
    else if g.fuel > start + step {color.a = 255} 
    else {color.a = u8((g.fuel-start) / step * 255)}
    start += step
    rl.DrawCircle(screen_width/2 - off*10, off2, size, color)


    // OUtlines
    //center Circle
    color = rl.GRAY
    rl.DrawCircleLines(screen_width/2,  off2, size *0.75, color)
    //4
    rl.DrawCircleLines(screen_width/2 + off, off2, size, color)
    rl.DrawCircleLines(screen_width/2 - off, off2, size, color)
    rl.DrawCircleLines(screen_width/2 + off*2, off2, size, color)
    rl.DrawCircleLines(screen_width/2 - off*2, off2, size, color)
    //6
    rl.DrawCircleLines(screen_width/2 + off*3, off2, size, color)
    rl.DrawCircleLines(screen_width/2 - off*3, off2, size, color)
    //8
    rl.DrawCircleLines(screen_width/2 + off*4, off2, size, color)
    rl.DrawCircleLines(screen_width/2 - off*4, off2, size, color)
    //12
    rl.DrawCircleLines(screen_width/2 + off*5, off2, size, color)
    rl.DrawCircleLines(screen_width/2 - off*5, off2, size, color)
    rl.DrawCircleLines(screen_width/2 + off*6, off2, size, color)
    rl.DrawCircleLines(screen_width/2 - off*6, off2, size, color)
    //20
    rl.DrawCircleLines(screen_width/2 + off*7, off2, size, color)
    rl.DrawCircleLines(screen_width/2 - off*7, off2, size, color)
    rl.DrawCircleLines(screen_width/2 + off*8, off2, size, color)
    rl.DrawCircleLines(screen_width/2 - off*8, off2, size, color)
    rl.DrawCircleLines(screen_width/2 + off*9, off2, size, color)
    rl.DrawCircleLines(screen_width/2 - off*9, off2, size, color)
    rl.DrawCircleLines(screen_width/2 + off*10, off2, size, color)
    rl.DrawCircleLines(screen_width/2 - off*10, off2, size, color)

    if g.fling.fuel_locked {
        rl.DrawCircleLines(screen_width/2,  off2-6, size *0.25, rl.BLACK)
        size_lock :i32 = i32(size *0.7)
        half_lock :i32 = size_lock / 2
        rl.DrawRectangle(screen_width/2 - half_lock, off2 - half_lock + 3, size_lock, size_lock-1, rl.BLACK)
    }
}
