# install pyttsx and libgcc
import pyttsx
import time
time.sleep(1)

def obj_pairing(line):
    obj, depth = line.strip().split(',')
    return [obj, float(depth)/100]

def sentence_synth(obj_label, obj_depth, side_margin=0, dist_unit='meters'):
    obj_label = obj_label
    obj_depth = obj_depth

    sntnc = 'There is a %s %.1f %s ahead.'%(obj_label, obj_depth, dist_unit)

    if side_margin > 0:
        sntnc += (' %.1f %s to your right.'%(side_margin, dist_unit))
    elif side_margin < 0:
        sntnc += (' %.1f %s to your left.'%(side_margin, dist_unit))
    return sntnc

# sound = engine.getProperty('voices')
# engine.setProperty('voice', sound[16].id)

path = '/tmp/'
while True:
    with open(path + 'depth.txt') as file_object:
        lines = file_object.readlines()

    obj_dict = dict(map(obj_pairing, lines))
    obj_label, obj_depth = sorted(obj_dict.items(), key=lambda x: x[1])[0]

    engine = pyttsx.init()
    engine.say(sentence_synth(obj_label, obj_depth))
    engine.runAndWait()
    engine = None

    time.sleep(2)
