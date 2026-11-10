"""Pure geometric scan-plane sampling. Does not predict real sensor detection."""
import json
import math
from pathlib import Path


def segment_hits_box(a, b, box):
    lo,hi=0.0,1.0
    for j in (0,1):
        d=b[j]-a[j]
        if abs(d)<1e-12:
            if not box[j][0]<=a[j]<=box[j][1]: return False
        else:
            u,v=(box[j][0]-a[j])/d,(box[j][1]-a[j])/d
            lo=max(lo,min(u,v));hi=min(hi,max(u,v))
            if hi<lo:return False
    return hi>1e-8 and lo<1-1e-8


def coverage(layout):
    prof=layout['profile']; half=prof['nominal_section']/2
    obstacles=[((x-half,x+half),(y-half,y+half)) for x,y in prof['main_columns_xy']]
    sensors=layout['lidars'];results=[]
    # Nested rectangular perimeters outside a conservative wheel/deck footprint.
    # Same-height pillar occlusion is included. No beams through solid columns.
    for margin in (50,200,500,1000):
        wx,ly=242.1+margin,275+margin
        pts=[]
        for i in range(101):
            t=-1+2*i/100
            pts.extend(((wx,t*ly),(-wx,t*ly),(t*wx,ly),(t*wx,-ly)))
        missing=[]
        for p in pts:
            seen=False
            for s in sensors:
                a=(s['x'],s['y'])
                bearing=math.degrees(math.atan2(p[0]-a[0],p[1]-a[1]))
                diff=(bearing-s['heading_deg']+180)%360-180
                if abs(diff)<=135+1e-8 and not any(segment_hits_box(a,p,o) for o in obstacles):
                    seen=True;break
            if not seen:missing.append(p)
        results.append({'margin_mm':margin,'samples':len(pts),'visible':len(pts)-len(missing),'missed':missing})
    return {'method':'2D sample rays; nominal270deg FOV; main-column occlusion; conservative wheel/deck perimeter',
            'scan_z_mm':[s['scan_z'] for s in sensors], 'perimeters':results,
            'limitations':'No real optical housing origin, glass, range accuracy, cable/cover/arm occlusion, floor obstacles or safety validation.'}


if __name__=='__main__':
    layout=json.loads(Path(__file__).with_name('layout.json').read_text(encoding='utf-8'))
    print(json.dumps(coverage(layout),indent=2))
