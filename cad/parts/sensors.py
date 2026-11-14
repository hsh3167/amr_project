import math
from .common import *
from .structure import column,end_bolt,bolts_record

def lidars(d):
    r=[];lidar=d.parts['lidar'];mount=d.layout['lidar_mount']
    for pose in d.layout['lidars']:
        n=pose['name'];x=pose['x'];y=pose['y'];z0=pose['bbox_bottom_z'];pt=mount['plate_thickness'];yaw=-pose['heading_deg']
        place=lambda obj:rz(obj,yaw).translate((x,y,0))
        sensorplate=place(box(mount['plate_width'],mount['plate_depth'],pt,0,0,z0-pt/2))
        sx=pose['support_x'];sy=pose['support_y'];length=math.hypot(x-sx,y-sy)+25
        bridge=rz(box(length,25,pt,0,0,z0-pt/2),math.degrees(math.atan2(y-sy,x-sx))).translate(((x+sx)/2,(y+sy)/2,0))
        bracket=sensorplate+bridge
        if d.variant!='baseline':
            # Rev13: the support column moved clear of the main column; the bridge is bolted to its top.
            bracket=bracket-cylz(2.75,pt+2,sx,sy,z0-pt-1)
        r.append(PartRecord('60_diagonal_lidar_mount_CONCEPT_'+n,'lidars',bracket,(.28,.43,.54)))
        r.append(column(d,sx,sy,d.p['base_bottom_z']+d.p['base_thickness'],z0-pt,'64_2020_Al_lidar_column_'+n,'lidars'))
        if d.variant!='baseline':
            shapes=[end_bolt(sx,sy,d.p['base_bottom_z'],d.p['base_thickness'],True),end_bolt(sx,sy,z0,pt,False)]
            r.append(bolts_record('65_lidar_column_bolts_CB5-12_'+n,'lidars',shapes,2))
        r.append(PartRecord('61_SICK_TiM571_envelope_base_'+n,'lidars',place(box(lidar['width'],lidar['depth'],36,0,0,z0+18)),(.85,.88,.90),'drawing_envelope'))
        # Rev12b: the optical envelope stops under the 3mm scanner-top disc instead of overlapping it.
        top=3 if d.variant!='baseline' else 0
        r.append(PartRecord('62_SICK_TiM571_optical_envelope_'+n,'lidars',place(box(61,60,lidar['height']-36-top,0,0,z0+36+(lidar['height']-36-top)/2)),(.09,.16,.21),'drawing_envelope'))
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
        adapter=box(cm['plate_width'],cm['plate_depth'],cm['plate_thickness'],x,cm['plate_y'],cm['plate_center_z'])
        if d.variant!='baseline':adapter=adapter-cylz(2.75,cm['plate_thickness']+2,x,cm['profile_y'],cm['plate_center_z']-cm['plate_thickness']/2-1)
        r.append(PartRecord('71_camera_adapter_CONCEPT_'+n,'cameras',adapter,(.67,.72,.80)))
        if d.variant!='baseline':
            # Rev13: wedge from the flat adapter up to the tilted camera underside (camera frame as in vendor.camera_shape).
            from build123d import Location,Rotation
            frame=Location((x,y,z))*Rotation(0,0,-side*p['camera_yaw_deg'])*Rotation(-p['camera_pitch_down_deg'],0,0)
            local=r[-2].shape.moved(frame.inverse()).bounding_box()
            top=cm['plate_center_z']+cm['plate_thickness']/2
            block=box(local.size.X-4,local.size.Y-4,60,local.center().X,local.center().Y,local.min.Z-30).moved(frame)
            wedge=block&box(cm['plate_width'],cm['plate_depth'],60,x,cm['plate_y'],top+30)
            r.append(PartRecord('71_camera_wedge_CONCEPT_'+n,'cameras',wedge,(.80,.66,.40),'concept_support',
                                {'material':'3D-printed wedge; camera M2 screws pitch 80 not drawn'}))
        r.append(column(d,x,cm['profile_y'],cm['profile_bottom_z'],p['top_bottom_z'],'72_2020_Al_camera_column_'+n,'cameras'))
        if d.variant!='baseline':
            # Rev13: the hanger is bolted to the deck above and carries the adapter plate below.
            under=cm['plate_center_z']-cm['plate_thickness']/2
            shapes=[end_bolt(x,cm['profile_y'],p['top_bottom_z']+p['top_thickness'],p['top_thickness'],False),
                    end_bolt(x,cm['profile_y'],under,cm['plate_thickness'],True)]
            r.append(bolts_record('73_camera_column_bolts_CB5-12_'+n,'cameras',shapes,2))
    return r
