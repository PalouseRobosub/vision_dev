from sloth.annotations.container import AnnotationContainer

import os
import struct
import imghdr

class DarknetContainer(AnnotationContainer):

    def get_image_size(self, fname):
        print("Getting image size of {}".format(fname))
        '''Determine the image type of fhandle and return its size.
        from draco'''
        
        try:
            with open(fname, 'rb') as fhandle:
                print("fhandle: {}".format(fhandle))
                head = fhandle.read(24)
                print("Head: {}".format(head))
                if len(head) != 24:
                    return
                if imghdr.what(fname) == 'png':
                    check = struct.unpack('>i', head[4:8])[0]
                    if check != 0x0d0a1a0a:
                        return
                    width, height = struct.unpack('>ii', head[16:24])
                elif imghdr.what(fname) == 'gif':
                    width, height = struct.unpack('<HH', head[6:10])
                elif imghdr.what(fname) == 'jpeg':
                    try:
                        fhandle.seek(0) # Read 0xff next
                        size = 2
                        ftype = 0
                        while not 0xc0 <= ftype <= 0xcf:
                            fhandle.seek(size, 1)
                            byte = fhandle.read(1)
                            while ord(byte) == 0xff:
                                byte = fhandle.read(1)
                            ftype = ord(byte)
                            size = struct.unpack('>H', fhandle.read(2))[0] - 2
                        # We are at a SOFn block
                        fhandle.seek(1, 1)  # Skip `precision' byte.
                        height, width = struct.unpack('>HH', fhandle.read(4))
                    except Exception: #IGNORE:W0703
                        return
                else:
                    return
                return width, height
        except IOError:
            print("Failed to open file: {}".format(fname))
            return
    """
    Containter which writes annotations in the darknet format. This file
        will need to be parsed and split to use
    """

    def parseFromFile(self, fname):
        """
        Overwritten to read darknet format
        """
        labels = {}
        annotations = []
        parentDir = os.path.split(fname)[0] + "/"

        with open(fname, "r") as f:
            while True:
                line = f.readline().rstrip()
                if '---' in line and len(labels) > 0:
                    # Stop adding to list
                    break
                elif '---' not in line:
                    data = line.split(":")
                    print (data)
                    labels[int(data[1])] = data[0]

            #All labels loaded
            tmp = {}
            for line in f:
                if ">>" in line:
                    _,filename = line.split(" ")
                    tmp["filename"] = filename[1:-2]
                    tmp["class"] = "image"
                    tmp["annotations"] = []
                elif "<<" in line:
                    if not tmp['annotations']:
                        tmp["unlabeled"] = True
                    annotations.append(tmp)
                    tmp = {}
                elif len(line) > 0:
                    data = line.split(" ")
                    label = {}
                    label["class"] = labels[int(data[0])]
                    
                    path = parentDir + tmp['filename']
                    size = self.get_image_size(path)
                    if size is None:
                        print("Invalid size")
                        tmp = {}
                        continue
                    label["height"] = data[3] * size[0]
                    label["width"] = data[4] * size[0]
                    label["x"] = data[1] * size[0]
                    label["y"] = data[2] * size[1]

        return annotations

    
    def serializeToFile(self, fname, annotations):
        """
        Overwritten to write darknet files
        """
        print("Writing to file: {}".format(fname))
        parentDir = os.path.split(fname)[0]
        parentDir = parentDir + ("/" if parentDir else "")
        with open(fname, "w") as f:
            print("File open")
            labels = []
            for an in annotations:
                print("Using annotation: {}".format(an['annotations']))
                for l in an['annotations']:
                    if l['class'] not in labels:
                        print("Adding class: {}".format(l['class']))
                        labels.append(l['class'])
                        print("Labels: {}".format(labels))
            print ("Created class list")

            # Write class number to label conversion header
            f.write("---\n")
            print("Labels: {}".format(labels))
            for i, item in enumerate(labels):
                print("Writing pair: {}:{}".format(item, i))
                f.write("{} : {}\n".format(item, i))
                print("Wrote pair to file")
            f.write("---\n")

            print("Wrote class labels")
            # Write each file's annotations
            for an in annotations:
                f.write(">> \"" + an['filename'] + "\"\n")
                print("Using parent Dir: {}".format(parentDir))
                print("Getting image size of: {}".format(an["filename"]))
                path = parentDir + an['filename']
                size = self.get_image_size(path)
                print("Size: {}".format(size))
                for label in an['annotations']:
                    print("Adding label: {}".format(label))
                    dw = 1.0/size[0]
                    dh = 1.0/size[1]
                    x = (label['width'] / 2.0) + label['x']
                    y = (label['height'] / 2.0) + label['y']
                    x = x * dw
                    w = label['width'] * dw
                    y = y * dh
                    h = label['height'] * dh
                    print("Writing label to file")
                    f.write("{} {} {} {} {}\n".format(labels.index(label['class']), x, y, w, h))
                    print("Wrote to file")
                f.write("<<\n")
            print("Wrote annotations")
        print("Finished writing")
        return
