import cv2
import numpy
import robotsio
import struct
import yarp


def main():

    yarp.Network.init()

    images_in = robotsio.BufferedPortYarpImageOfMonoFloat()
    images_in.open('/hyperpcr/depth:i')

    width = # set width here
    height = # set height here

    depth_buffer = bytearray(numpy.zeros((height, width, 1), dtype = numpy.float32))
    depth_image = yarp.ImageFloat()
    depth_image.resize(width, height)
    depth_image.setExternal(depth_buffer, width, height)

    mask_buffer = bytearray(numpy.zeros((height, width, 1), dtype = numpy.uint8))
    mask_image = yarp.ImageMono()
    mask_image.resize(width, height)
    mask_image.setExternal(mask_buffer, width, height)

    images_data = None
    while images_data is None:
        images_data = images_in.read(False)

        if images_data is not None:
            mask_image.copy(images_data.image_mono)
            depth_image.copy(images_data.image_float)

            depth_frame = numpy.frombuffer(depth_buffer, dtype=numpy.float32).reshape(height, width)
            mask_frame = numpy.frombuffer(mask_buffer, dtype=numpy.uint8).reshape(height, width)

            # save
            cv2.imwrite('./mask.png', mask_frame)

            depth_file = open('depth.float', "wb")
            depth_file.write(struct.pack('=Q', width))
            depth_file.write(struct.pack('=Q', height))
            depth_file.write(depth_frame.astype('float32', order='C').tobytes())
            depth_file.close()

            print('received')

            break

if __name__ == '__main__':
    main()
