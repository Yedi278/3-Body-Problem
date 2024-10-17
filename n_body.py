import vpython as vp
from random import randint

def axis_arrow():
    '''
    This function creates the x, y, z axis arrows

    Returns:
    x_axis: vpython arrow object
    y_axis: vpython arrow object
    z_axis: vpython arrow object
    '''
    _scale = 1000
    x_axis = vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(_scale, 0, 0), color=vp.color.white, shaftwidth=5, headwidth=10)
    y_axis = vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(0, _scale, 0), color=vp.color.white, shaftwidth=5, headwidth=.5)
    z_axis = vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(0, 0, _scale), color=vp.color.white, shaftwidth=5, headwidth=.5)

    return x_axis, y_axis, z_axis


def init_system(n:int) -> list:
    '''
    This function initializes the n-body system's variables and graphic space
    '''
    vp.scene.autoscale = True
    vp.scene.width = 1200
    vp.scene.height = 700
    
    vp.scene.center = vp.vector(10,10, 0)

    body_list = list() # list of body objects (dictionaries)
    colors = [vp.color.red, vp.color.blue, vp.color.green, vp.color.yellow, vp.color.orange, vp.color.purple, vp.color.white]

    for i in range(n):
        _color = colors[i]
        _mass = randint(10, 30)
        body_list.append({'obj':vp.sphere(pos=vp.vector(i*100, i*100, i*100), radius=_mass, color=_color, make_trail=True, trail_type='curve', trail_color=_color, trail_radius=3),
                          'mass':_mass,
                          'arrow':vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(0, 0, 0), color=vp.color.white, shaftwidth=3, headwidth=3)})
        body_list[i]['obj'].velocity = vp.vector(0, 0, 0)

    return body_list

def dot(v1:vp.vector, v2:vp.vector):
    ''' Eucledian dot product of two vectors '''
    return (v1.x*v2.x) + (v1.y*v2.y) + (v1.z*v2.z)

def update_system(objects:list, dt=0.1):
    '''
    This function updates the system of bodies in the graphic space
    using Euler's method for numerical integration and resolves collisions with elastic collisions and 0.01% energy loss
    '''
    G = 300

    for i in objects:
        for j in objects:
            if i != j:
                r = vp.vector(j['obj'].pos - i['obj'].pos)
                a_i_j = G * j['mass'] * r / vp.mag(r)**3
                a_j_i = - G * i['mass'] * r / vp.mag(r)**3

                i['obj'].velocity += a_i_j * dt
                j['obj'].velocity += a_j_i * dt

                i['obj'].pos += i['obj'].velocity * dt
                j['obj'].pos += j['obj'].velocity * dt
                
                r = vp.vector(j['obj'].pos - i['obj'].pos)

                if r.mag < i['obj'].radius + j['obj'].radius:
                    i['obj'].velocity -= 2 * dot(i['obj'].velocity, r) * r / dot(r, r)
                    j['obj'].velocity -= 2 * dot(j['obj'].velocity, r) * r / dot(r, r)
                    
                    i['obj'].velocity *= 0.9999
                    j['obj'].velocity *= 0.9999

                    i['obj'].pos -= 0.5 * abs(i['obj'].radius + j['obj'].radius - r.mag) * r.norm()
                    j['obj'].pos += 0.5 * abs(i['obj'].radius + j['obj'].radius - r.mag) * r.norm()

    for i in objects:
        i['arrow'].pos = i['obj'].pos + i['obj'].radius * i['obj'].velocity.norm()
        i['arrow'].axis = 5*i['obj'].velocity

def update_button(evt):
    ''' This function updates the global run variable for pausing the simulation '''
    global run
    run = not run

def set_camera_follower(objects:list) -> int:
    ''' This function sets the camera to follow the object with the maximum mass '''
    max_mass_obj = objects[0]
    for i in objects:
        if i['mass'] > max_mass_obj['mass']:
            max_mass_obj = i
    
    vp.scene.camera.follow(max_mass_obj['obj'])

if __name__ == '__main__':

    # set the axis in the plane
    axis_arrow()
    # create a pause button
    button = vp.button(text='Pause', bind=update_button)

    # initialize the system
    body_list = init_system(3)
    # set the camera to follow the object with the maximum mass
    set_camera_follower(body_list)
    
    # set some initial parameters for the objects
    body_list[0]['obj'].velocity = vp.vector(5, 0, 0)
    body_list[1]['obj'].velocity = vp.vector(0, 5, 0)
    body_list[2]['obj'].velocity = vp.vector(0, 0, 5)

    run = True
    while True:

        if run:
            update_system(body_list, dt=0.01)
            vp.rate(1000)
        else:
            pass