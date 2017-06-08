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

if __name__ == "__main__":
    path = '/tmp/'
    while True:
        with open(path + 'depth.txt') as file_object:
            lines = file_object.readlines()

        obj_dict = dict(map(obj_pairing, lines))
        if(len(obj_dict) == 2):
            pass
            # TODO: joy stick
        elif obj_dict.keys()[0] == "hand":
            pass
            # TODO: no alert
        else:
            pass
            # TODO: raise your hand

        engine = pyttsx.init()
        rate = engine.getProperty('rate')
        engine.setProperty('rate', rate-20)
        engine.say(sentence_synth(obj_label, obj_depth, out_unit='feet'))
        engine.runAndWait()
        engine = None
        time.sleep(2)
