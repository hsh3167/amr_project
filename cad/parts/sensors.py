import math
from .common import *
from .structure import column

def lidars(d):
    r=[];lidar=d.parts['lidar'];mount=d.layout['lidar_mount']
    for pose in d.layout['lidars']:
        n=pose['name'];x=pose['x'];y=pose['y'];z0=pose['bbox_bottom_z'];pt=mount['plate_thickness'];yaw=-pose['heading_deg']
        place=lambda obj:rz(obj,yaw).translate((x,y,0))
        sensorplate=place(box(mount['plate_width'],mount['plate_depth'],pt,0,0,z0-pt/2))
        sx=pose['support_x'];sy=pose['support_y'];length=math.hypot(x-sx,y-sy)+25
        bridge=rz(box(length,25,pt,0,0,z0-pt/2),math.degrees(math.atan2(y-sy,x-sx))).translate(((x+sx)/2,(y+sy)/2,0))
        r.append(PartRecord('60_diagonal_lidar_mount_CONCEPT_'+n,'lidars',sensorplate+bridge,(.28,.43,.54)))
        r.append(column(d,sx,sy,d.p['base_bottom_z']+d.p['base_thickness'],z0-pt,'64_2020_Al_lidar_column_'+n,'lidars'))
        r.append(PartRecord('61_SICK_TiM571_envelope_base_'+n,'lidars',place(box(lidar['width'],lidar['depth'],36,0,0,z0+18)),(.85,.88,.90),'drawing_envelope'))
        r.append(PartRecord('62_SICK_TiM571_optical_envelope_'+n,'lidars',place(box(61,60,lidar['height']-36,0,0,z0+36+(lidar['height']-36)/2)),(.09,.16,.21),'drawing_envelope'))
        r.append(PartRecord('63_SICK_scanner_top_'+n,'lidars',cylz(22,3,x,y,z0+lidar['height']-3),(.20,.80,.79),'drawing_envelope'))
    return r

def cameras(d):
    r=[];p=d.p;cam=d.parts['camera'];cm=d.layout['camera_mount']
    for side in (-1,1):
        n='LEFT' if side<0 else 'RIGHT';x=side*p['camera_x'];y=p['camera_y'];z=p['camera_z']
        if d.variant=='baseline':
            body=box(cam['width'],cam['depth'],cam['height']).rotate(Axis.X,-p['camera_pitch_down_deg'])
            body=rz(body,-side*p['camera_yaw_deg']).translate((x,y,z))
            r.append(PartRecord('70_Orbbec_Astra_Mini_Pro_MX6000_'+n,'cameras',body,(.65,.50,.82),'drawing_envelope'))
        else:
            from .vendor import camera_records
            r.extend(camera_records(d,side))
        r.append(PartRecord('71_camera_adapter_CONCEPT_'+n,'cameras',box(cm['plate_width'],cm['plate_depth'],cm['plate_thickness'],x,cm['plate_y'],cm['plate_center_z']),(.67,.72,.80)))
        r.append(column(d,x,cm['profile_y'],cm['profile_bottom_z'],p['top_bottom_z'],'72_2020_Al_camera_column_'+n,'cameras'))
    return r
