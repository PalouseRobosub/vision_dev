from sloth.annotations.container import AnnotationContainer

import os
import struct
import imghdr

class VOCContainer(AnnotationContainer):

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
    # TODO
    def parseFromFile(self, fname):
        """
        Overwritten to read VOC format
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
        print "writing to file: {}".format(fname)
        parentDir = os.path.split(fname)[0]+"/"

        # write each file's annotations
        with open(fname, "w") as f:
            print 'opened file'
            for annotation in annotations:
                f.write(" >> \"" + annotation['filename'] + "\"\n") # write annotation filename to begin this file segment
                print "adding file: {}".format(annotation['filename'])

                # write xml header
                f.write("<annotation>\n")
                
                # folder
                f.write("   <folder>{}</folder>\n".format("combine"))

                # filename
                f.write("   <filename>{}</filename>\n".format(annotation['filename']))

                # source
                f.write("   <source>\n")
                f.write("       <database>{}</database>\n".format("The Robosub Database"))
                f.write("       <annotation>{}</annotation>\n".format("PASCAL VOC2007"))
                f.write("       <image>{}</image>\n".format("none"))
                f.write("       <flickrid>{}</flickrid>\n".format("none"))
                f.write("   </source>\n")
                
                # owner
                f.write("   <owner>\n")
                #f.write("       <flickrid>{}</flickrid>\n".format("INSERT"))
                f.write("       <name>{}</name>\n".format("Palouse Robosub"))
                f.write("   </owner>\n")

                # size
                f.write("   <size>\n")
                f.write("       <width>{}</width>\n".format("1384"))
                f.write("       <height>{}</height>\n".format("1032"))
                f.write("       <depth>{}</depth>\n".format("3"))
                f.write("   </size>\n")
                # segmented

                # write labels
                for label in annotation['annotations']:
                    print "    adding label: {}".format(label)

                    # gather data
                    lblClass = label['class']
                    x = label['x']
                    y = label['y']
                    width = label['width']
                    height = label['height']

                    xmin = x
                    ymin = y
                    xmax = x + width
                    ymax = y + height

                    # write out data to file
                    f.write("   <object>\n")
                    f.write("       <name>{}</name>\n".format(lblClass))
                    f.write("       <pose>{}</pose>\n".format("Unspecified"))
                    f.write("       <truncated>{}</truncated>\n".format(1))
                    f.write("       <difficult>{}</difficult>\n".format(0))
                    f.write("       <bndbox>\n")
                    f.write("           <xmin>{}</xmin>\n".format(xmin))
                    f.write("           <ymin>{}</ymin>\n".format(ymin))
                    f.write("           <xmax>{}</xmax>\n".format(xmax))
                    f.write("           <ymax>{}</ymax>\n".format(ymax))
                    f.write("       </bndbox>\n")
                    f.write("   </object>\n")

                # write xml footer
                f.write("</annotation>\n")
                f.write("<<\n")
                print "    finished file"

    
