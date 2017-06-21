# install pyttsx and libgcc
import pyttsx
import time

def obj_pairing(line):
    '''
    The input is line consisting of:
    lable,distance,x,y
    '''
    obj, info = line.strip().split(',', 1)
    info = map(lambda x:float(x), info.split(','))
    return (obj, tuple(info))

def sentence_synth(obj_label, obj_depth, side_margin=0, out_unit='feet'):
    '''
    This function receives the info regarding the closest object in screen
        obj_label -- object label
        obj_depth -- object depth
        side_margin -- optional, states the horizontal location, default 0
        out_unit -- output distance unit, can be meters or feet, default meters
    (Assuming the input distance unit is 'meter'.)
    '''
    obj_label = obj_label
    obj_depth = obj_depth

    if out_unit.lower() == 'meters':
        obj_depth /= 3.28084
        side_margin /= 3.28084
    sntnc = 'A %s %.1f %s ahead.'%(obj_label, obj_depth, out_unit)

    if side_margin > 0:
        sntnc += (' %.1f %s to your right.'%(side_margin, out_unit))
    elif side_margin < 0:
        sntnc += (' %.1f %s to your left.'%(side_margin, out_unit))
    return sntnc

# sound = engine.getProperty('voices')
# engine.setProperty('voice', sound[16].id)

def feedback(d):
    th = 0.1 # threshold to decide whether two objects are in same position
    if(len(d) == 2): # joy stick
        stretch = (d['door handle'][0] - d['hand'][0])> 0.3
        delta_x = d['door handle'][1] - d['hand'][1]
        delta_y = d['door handle'][2] - d['hand'][2]
        if abs(delta_x > delta_y):
            if delta_x > th:
                return "right" + (" and stretch" if stretch else "")
            elif delta_x < -th:
                return "left" + (" and stretch" if stretch else "")
            else:
                pass # else goto y decision
        if delta_y > th:
            return "up" + (" and stretch" if stretch else "")
        elif delta_y < -th:
            return "down" + (" and stretch" if stretch else "")
        else:
            return "stretch" if stretch else ""
    elif d.keys()[0] == "hand" or len(d)>2: # no alert
        return ""
    else: # only door handle
        fb = "Door handle is "
        distance = d['door handle'][0]
        x = d['door handle'][0]
        y = d['door handle'][1]
        if y > th:
            fb += 'up '
        elif y < -th:
            fb += 'down '
        if x > th:
            fb += 'right '
        elif x < -th:
            fb += 'left '
        fb += "%.1f" % distance
        fb += " feet from you"
        return fb

if __name__ == "__main__":
    path = '/tmp/'
    DEBUG = True
    while True:
        with open(path + 'depth.txt') as file_object:
            lines = file_object.readlines()

        obj_dict = dict(map(obj_pairing, lines))
        if DEBUG: print "DEBUG,obj_dict:", obj_dict

        fb = feedback(obj_dict)
        if DEBUG: print "DEBUG,fb:", fb
        if not fb:
            time.sleep(0.1)
            continue

        engine = pyttsx.init()
        rate = engine.getProperty('rate')
        engine.setProperty('rate', rate-20)
        # engine.say(sentence_synth(obj_label, obj_depth, out_unit='feet'))
        engine.say(fb)
        engine.runAndWait()
        engine = None
        time.sleep(2)
