import numpy as np
from PIL import Image, ImageDraw
from bs4 import BeautifulSoup
import os
from pprint import pprint
import sys
import cv2


try:
    #python 2 imports
    import ConfigParser
    config = ConfigParser.RawConfigParser()
except:
    # python 3 imports
    import configparser
    config = configparser.ConfigParser()

config.read('conf.txt')

home_dir = config.get('directories', 'home_dir')

# input data directories
xml_dir = home_dir + "annotations_init/xmls/"
img_dir = home_dir + "images_init/"

# output data directories 
aug_xml_dir = home_dir + "annotations/xmls/"
aug_img_dir = home_dir + "images/"


dmin = 10 # minimal distance from bounding box till border of original image
d = 10 # width of margin which to multiply


times_added = 40 # how many times margins should be added
filter_window = 21 # filter window for blurring

def add_sides_only(img, xmin, ymin, xmax, ymax, xmorig, ymorig, xMorig, yMorig):
    """
    This funtion copies image margins reshapes the a bit and adds to the image sides
    input:
    img - image representative object from pillow
    xmin, ymin, xmax, ymax - bounding box parameters
    xmorig, ymorig, xMorig, yMorig - box which represents original location in the augmented image 

    output:
    img1 - augmented image as object from pillow package
    is_ok - which image sides are ok to be used for augmentation
    xmin, ymin, xmax, ymax - bounding box parameters respective to output image
    xmorig, ymorig, xMorig, yMorig - box which represents original location in the augmented image respective to output image
    """
    w, h = img.size
    
    # indicators whether respecitve side has a margin to be multiplied
    is_ok = [
        ymin > dmin, # upper
        ymax < h-dmin, # lower
        xmin > dmin, # left
        xmax < w-dmin # right
    ]
    num_up_down = sum(is_ok[0:2])
    num_right_left = sum(is_ok[2:4])

    img1 = np.asarray(img.copy())

    # adds marign to each side and modifies bounding box and original image box if needed
    if is_ok[0]:
        newsize = (w,d)
        margin = np.asarray(img.crop([0,0,w,d]))#.resize(newsize,Image.ANTIALIAS)
        img1 = np.concatenate([margin, img1],axis=0)
        ymin, ymax = ymin + d, ymax + d
        ymorig, yMorig = ymorig + d, yMorig + d

    if is_ok[1] :
        newsize = (w,d)
        margin = np.asarray(img.crop([0,h-d,w,h]))#.resize(newsize,Image.ANTIALIAS)
        img1 = np.concatenate([img1, margin],axis=0)

    if is_ok[2]:
        newsize = (d, h + num_up_down * d)
        margin = np.asarray(img.crop([0, 0, d, h]).resize(newsize, Image.ANTIALIAS))
        img1 = np.concatenate([margin, img1],axis=1)
        xmin, xmax = xmin + d, xmax + d
        xmorig, xMorig = xmorig + d, xMorig + d

    if is_ok[3]:
        newsize = (d, h + num_up_down * d)
        margin = np.asarray(img.crop([w-d, 0, w, h]).resize(newsize, Image.ANTIALIAS))
        img1 = np.concatenate([img1,margin],axis=1)
    
    return img1, is_ok, xmin, ymin, xmax, ymax, xmorig, ymorig, xMorig, yMorig 



def add_margs(img, xmin, ymin, xmax, ymax, xmorig=None, ymorig=None, xMorig=None, yMorig=None, orig_sizes=None):
    """
    This function is a wrapper with added argument transformations for add_sides_only
    input:
    img - image representative object from pillow
    xmin, ymin, xmax, ymax - bounding box parameters
    xmorig, ymorig, xMorig, yMorig - box which represents original location in the augmented image
    orig_sizes - size of original image

    output:
    img1 - augmented image as object from pillow package
    is_ok - which image sides are ok to be used for augmentation
    xmin, ymin, xmax, ymax - bounding box parameters respective to output image
    xmorig, ymorig, xMorig, yMorig - box which represents original location in the augmented image respective to output image
    orig_sizes - size of original image
    """
    w, h = img.size
    if xmorig==None or ymorig==None or xMorig==None or yMorig==None:
        xmorig, ymorig, xMorig, yMorig = 0, 0, w, h

    # applies image transformation once
    img1, is_ok, xmin1, ymin1, xmax1, ymax1, xmorig1, ymorig1, xMorig1, yMorig1 = \
    add_sides_only(img, xmin, ymin, xmax, ymax, xmorig, ymorig, xMorig, yMorig)
    

    ratio = 2.0 # maximal propotion between original and augmented bounding boxes (so that apple don't get deformed too much)
    if orig_sizes is not None:
        new_orig_width = float(xMorig1-xmorig1)
        new_orig_height = float(yMorig1-ymorig1)
        w, h = float(orig_sizes[0]), float(orig_sizes[1])
        coef = new_orig_height * w / (h * new_orig_width)
        # if bounding box gets deformed too much in this iteration, augmentatino of this image stops
        if coef > ratio or coef < 1/ratio :
            is_ok = [False]*4
            return img, is_ok, xmin, ymin, xmax, ymax, xmorig, ymorig, xMorig, yMorig

    img1 = Image.fromarray(img1)
    w, h = img1.size

    # if h>1000 or w>1000:
    #     h1 = min(1000,h)
    #     w1 = min(1000,w)
    #     xmin1 = int(float(w1 * xmin1)/float(w))
    #     ymin1 = int(float(h1 * ymin1)/float(h))
    #     xmax1 = int(float(w1 * xmax1)/float(w))
    #     ymax1 = int(float(h1 * ymax1)/float(h))

    #     xmorig1 = int(float(w1 * xmorig1)/float(w))
    #     ymorig1 = int(float(h1 * ymorig1)/float(h))
    #     xMorig1 = int(float(w1 * xMorig1)/float(w))
    #     yMorig1 = int(float(h1 * yMorig1)/float(h))

    #     img1 = img1.resize((w1,h1),Image.ANTIALIAS)
    
    # if resolution of apple gets too low, then image augmentation stops
    dx = xmax1 - xmin1
    dy = ymax1 - ymin1
    if dx<20 or dy<20:
        is_ok = [False]*4
        return img, is_ok, xmin, ymin, xmax, ymax, xmorig, ymorig, xMorig, yMorig

    return img1, is_ok, xmin1, ymin1, xmax1, ymax1, xmorig1, ymorig1, xMorig1, yMorig1




def transform_image(img, xmin, ymin, xmax, ymax, times_added=[40,60]):
    """
    This function applies full transformation for one image.
    input:
    img - input image representation from pillow
    xmin, ymin, xmax, ymax - bounding box parameters
    times_added - list of how many times add_margs should be applied to original image

    output:
    outs - list of augmented images respective to times_added list
    """    
    orig_size = img.size  
    img_original = img.copy()
    
    max_times_added= max(times_added)
    outs = []

    for i in range(max_times_added):
        if i==0:
            rez = add_margs(img, xmin, ymin, xmax, ymax)
        else:
            rez = add_margs(img, xmin, ymin, xmax, ymax, xmorig, ymorig, xMorig, yMorig, orig_size)
        img, is_ok, xmin, ymin, xmax, ymax, xmorig, ymorig, xMorig, yMorig = rez
        # print("ymorig,yMorig ", ymorig, yMorig," d=",yMorig-ymorig," img.size ", img.size)
    
        if i+1 in times_added:
            w_orig, h_orig = img_original.size

            blur = cv2.GaussianBlur(np.asarray(img),(filter_window,filter_window),0)
            h_blur, w_blur = blur.shape[0:2]

            dx = xMorig - xmorig
            dy = yMorig - ymorig
            blur[ymorig:yMorig,xmorig:xMorig,:] = np.asarray(img_original.resize((dx,dy),Image.ANTIALIAS))

            outs.append([Image.fromarray(blur), xmin, ymin, xmax, ymax])
    
        if sum(is_ok)==0:
            break

    return outs



def change_filname(xml,tag, newname):
    """
    This function change content of a tag
    input:
    xml - tag from xml representation in BeautifulSoup (ATTENTION: this parameter is passed as reference)
    tag - tag name
    newname - new tag content
    """
    tag_str = "<" + tag + ">" + newname + "</" + tag + ">"
    xml.replace_with(BeautifulSoup(tag_str, "lxml").find_all(tag)[0])



fs = os.listdir(xml_dir)
# fs = [f for f in fs if "n07739125_10279" in f]

# loop goes through all images and respective xml files
for nr, f in enumerate(fs):

    print("Proportion of processed images: {}".format(float(nr)/float(len(fs))))
    # print(float(nr)/float(100))
    name = f.split(".")[0]
    # name = "n07739125_4157"

    xmlfile = xml_dir + name + ".xml"
    imgfile = img_dir + name + ".JPEG"

    aug_xmlfile = aug_xml_dir + name + ".xml"
    aug_imgfile = aug_img_dir + name + ".JPEG"

    xml = BeautifulSoup(open(xmlfile, 'r').read(), "lxml").annotation
    img = Image.open(imgfile)

    # saving original files
    with open(aug_xmlfile, 'w') as myfile:
        myfile.write(str(xml))
    img.save(aug_imgfile)

    xmin = int(xml.find("xmin").contents[0])
    ymin = int(xml.find("ymin").contents[0])
    xmax = int(xml.find("xmax").contents[0])
    ymax = int(xml.find("ymax").contents[0])

    # augmenting original image
    if len(xml.find_all("object"))==1:
        outs = transform_image(img, xmin, ymin, xmax, ymax, times_added=[40,60,80,140])
        for trains_id, out in enumerate(outs):
            name1 = name + "_" + str(trains_id)
            aug_xmlfile = aug_xml_dir + name1 + ".xml"
            aug_imgfile = aug_img_dir + name1 + ".JPEG"

            # modifying image
            img1, xmin1, ymin1, xmax1, ymax1 = out

            # modifying xml file
            xml1 = BeautifulSoup(str(xml), "lxml").annotation
            change_filname(xml1.filename,"filename",name1)
            change_filname(xml1.xmin,"xmin",str(xmin1))
            change_filname(xml1.ymin,"ymin",str(ymin1))
            change_filname(xml1.xmax,"xmax",str(xmax1))
            change_filname(xml1.ymax,"ymax",str(ymax1))

            # saving augmented data
            with open(aug_xmlfile, 'w') as myfile:
                myfile.write(str(xml1))
            # print("xmin1, ymin1, xmax1, ymax1  ",(xmin1, ymin1, xmax1, ymax1))
            # print("img1.size ",img1.size)
            # draw1 = ImageDraw.Draw(img1)
            # draw1.rectangle([xmin1, ymin1, xmax1, ymax1],outline=(255,0,0))

            # resizes images if their resolution too high
            w, h = img.size
            if w>1000:
                coef = float(w)/1000.0
                w = 1000
                h = int(float(h)/coef)
            if h>1000:
                coef = float(h)/1000.0
                h = 1000
                w = int(float(w)/coef)
            img1 = img1.resize((w,h),Image.ANTIALIAS)
            img1.save(aug_imgfile)

