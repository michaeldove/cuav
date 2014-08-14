#!/usr/bin/python
'''
Thumbnail image helpers.
Andrew Tridgell
May 2012
'''

import cv
from cuav.lib import cuav_util

def ExtractThumbs(img, count):
    '''extract thumbnails from a composite thumbnail image'''
    thumb_size = cuav_util.image_width(img) / count
    thumbs = []
    for i in range(count):
        thumb = cuav_util.SubImage(img, (i*thumb_size, 0, thumb_size, thumb_size))
        thumbs.append(thumb)
    return thumbs

def CompositeThumbnail(img, regions, thumb_size=100):
    '''extract a composite thumbnail for the regions of an image

    The composite will consist of N thumbnails side by side
    '''
    composite = cv.CreateImage((thumb_size*len(regions), thumb_size),8,3)
    for i in range(len(regions)):
        (x1,y1,x2,y2) = regions[i].tuple()
        midx = (x1+x2)/2
        midy = (y1+y2)/2

        if (x2-x1) > thumb_size or (y2-y1) > thumb_size:
            # we need to shrink the region
            rsize = max(x2+1-x1, y2+1-y1)
            src = cuav_util.SubImage(img, (midx-rsize/2,midy-rsize/2,rsize,rsize))
            thumb = cv.CreateImage((thumb_size, thumb_size),8,3)
            cv.Resize(src, thumb)
        else:
            x1 = midx - thumb_size/2
            y1 = midy - thumb_size/2
            thumb = cuav_util.SubImage(img, (x1, y1, thumb_size, thumb_size))
        cv.SetImageROI(composite, (thumb_size*i, 0, thumb_size, thumb_size))
        cv.Copy(thumb, composite)
        cv.ResetImageROI(composite)
    return composite
