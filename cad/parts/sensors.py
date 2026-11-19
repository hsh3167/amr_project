import math
from .common import *
from build123d import Compound
from .structure import column,end_bolt,bolts_record

def lidar_bracket(d,n,x,y,z0,pt,sensorplate):
    """Rev14: 3t bent bracket wrapping two faces of the nearest main 2020 column (2 bends, bend limit 3t).

    The sensor plate, an L-shaped deck around the column corner and two 32 mm down-flanges are one
    laser-cut part. Each flange carries 2x M5 bolts into T-nuts in the column slot (4 bolts total), so the
    LiDAR is held by the stiff main column (bolted to the base and the deck) instead of a single bolt.
    """
    q=d.layout['lidar_mount'];sx=1 if x>0 else -1;sy=1 if y>0 else -1
    cx,cy=next((a,b) for a,b in d.profile['main_columns_xy'] if a*sx>0 and b*sy>0)
    h=d.profile['nominal_section']/2;fx=cx+sx*h;fy=cy+sy*h;top=z0-pt;drop=q['flange_drop_mm'];reach=q['deck_reach_mm']
    up=q.get('flange_up_mm',0);lip=q.get('stiffening_lip_mm',0)
    def S(x0,x1,y0,y1,z0_,z1):return span(min(x0,x1),max(x0,x1),min(y0,y1),max(y0,y1),z0_,z1)
    deck=S(fx,fx+sx*reach,cy-sy*h,cy+sy*reach,top,z0)+S(cx-sx*h,fx,fy,cy+sy*reach,top,z0)
    plate=sensorplate+deck
    # Rev17: flanges go UP the column faces (deck to deck+up, 3 bolts each) instead of 32 mm down (2 bolts): the bolt
    # pattern spans 39 mm instead of 16 mm. Flanges stay below the scan plane.
    f0,f1=(top,z0+up) if up else (top-drop,top)
    flange_x=S(fx,fx+sx*pt,cy-sy*h,fy,f0,f1);flange_y=S(cx-sx*h,fx,fy,fy+sy*pt,f0,f1)
    bracket=plate+flange_x+flange_y
    if lip:
        # Rev17: downturned stiffening lips on the sensor plate edges (2 more bends), trimmed clear of the column.
        yaw=-next(p_['heading_deg'] for p_ in d.layout['lidars'] if p_['x']==x and p_['y']==y)
        pw,pd=q['plate_width'],q['plate_depth']
        lips=None
        for ly in (-1,1):
            b_=rz(box(pw,pt,lip,0,ly*(pd/2-pt/2),top-lip/2),yaw).translate((x,y,0))
            lips=b_ if lips is None else lips+b_
        for lx in (-1,1):
            b_=rz(box(pt,pd,lip,lx*(pw/2-pt/2),0,top-lip/2),yaw).translate((x,y,0))
            lips=lips+b_
        lips=lips-span(cx-h-4,cx+h+4,cy-h-4,cy+h+4,top-lip-1,z0+1)
        bracket=bracket+lips
    bolts=[];zs=[z0+8,z0+up/2+pt/2,z0+up-8] if up else [top-drop+8,top-8]
    for z in zs:
        bracket=bracket-cylx(2.75,pt+2,fx+sx*pt/2,cy,z)-cyly(2.75,pt+2,cx,fy+sy*pt/2,z)
        # Head outside the flange, shank through the slot mouth, T-nut in the slot channel (depth 2.1-4.4).
        bolts+=[cylx(4.25,5,fx+sx*(pt+2.5),cy,z)+cylx(2.5,pt+4.4,fx+sx*(pt-4.4)/2,cy,z),
                cyly(4.25,5,cx,fy+sy*(pt+2.5),z)+cyly(2.5,pt+4.4,cx,fy+sy*(pt-4.4)/2,z)]
        nuts=[box(2.3,9.6,10,fx-sx*3.25,cy,z)-cylx(2.5,4,fx-sx*3.25,cy,z),box(9.6,2.3,10,cx,fy-sy*3.25,z)-cyly(2.5,4,cx,fy-sy*3.25,z)]
        bolts+=nuts
    return [PartRecord('60_lidar_bracket_3t_'+n,'lidars',bracket,(.28,.43,.54),'concept_fabrication',
                       {'material':'S355 3t laser-cut; wraps two faces of the main column'+
                        (f'; Rev17: {up:g} mm up-flanges with 3 bolts each + {lip:g} mm stiffening lips on the sensor plate (6 bends)' if up else ', 2 bends')}),
            PartRecord('65_lidar_bracket_bolts_M5_Tnut_'+n,'lidars',Compound(bolts),(.20,.22,.25),'catalog_envelope',
                       {'sku':'M5x10 hex socket + M5 T-nut for 2020 slot 6','quantity':len(zs)*2})]


def pedestal_base_bolts(d,x,y):
    """(x, y) of the pedestal-to-base-plate bolts, from the main column next to the LiDAR."""
    q=d.layout['lidar_mount'];sx=1 if x>0 else -1;sy=1 if y>0 else -1
    cx,cy=next((a,b) for a,b in d.profile['main_columns_xy'] if a*sx>0 and b*sy>0)
    return [(cx+sx*a,cy+sy*b) for a,b in q['pedestal_base_bolts_rel']]


def mount_holes(d):
    """(x, y, d) base-plate holes for the Rev17b LiDAR pedestals."""
    if d.variant=='baseline':return []
    return [(px,py,5.5) for pose in d.layout['lidars'] for px,py in pedestal_base_bolts(d,pose['x'],pose['y'])]


def lidar_pedestal(d,n,x,y,z0,yaw):
    """Rev17b: one 3D-printed pedestal per LiDAR instead of the 3t sheet cantilever.

    Solid from the base-plate top up to the scanner bottom (43 mm deep) under the scanner and around the main
    column corner, two 6 mm walls 55 mm up the column faces (3 M5 T-nut bolts each) and 3 M5 bolts into the base
    plate. A deep printed block is far stiffer than a 3 mm cantilever plate, so the scanner does not ring."""
    from .suspension import screw,nut
    from .electronics import PRINT_NOTE
    q=d.layout['lidar_mount'];sx=1 if x>0 else -1;sy=1 if y>0 else -1
    cx,cy=next((a,b) for a,b in d.profile['main_columns_xy'] if a*sx>0 and b*sy>0)
    h=d.profile['nominal_section']/2;fx=cx+sx*h;fy=cy+sy*h
    base_bottom=d.p['base_bottom_z'];base_top=base_bottom+d.p['base_thickness']
    w=q['pedestal_wall_mm'];up=q['flange_up_mm'];reach=q['deck_reach_mm'];deck=q['pedestal_deck_mm']
    def S(x0,x1,y0,y1,z0_,z1):return span(min(x0,x1),max(x0,x1),min(y0,y1),max(y0,y1),z0_,z1)
    pad=rz(box(q['pedestal_pad'][0],q['pedestal_pad'][1],z0-base_top,0,0,(z0+base_top)/2),yaw).translate((x,y,0))
    body=pad+S(fx,fx+sx*reach,cy-sy*h,cy+sy*reach,base_top,z0)+S(cx-sx*h,fx,fy,cy+sy*reach,base_top,z0)
    body=body+S(fx,fx+sx*w,cy-sy*h,fy+sy*w,z0,z0+up)+S(cx-sx*h,fx,fy,fy+sy*w,z0,z0+up)
    body=body-S(cx-h-.2,cx+h+.2,cy-h-.2,cy+h+.2,base_top-1,z0+up+1)
    # column bolts: head outside the 6 mm wall, shank through the slot mouth, T-nut in the 2020 slot channel
    zs=[z0+8,z0+up/2,z0+up-8];bolts=[]
    for z in zs:
        body=body-cylx(2.75,w+2,fx+sx*w/2,cy,z)-cyly(2.75,w+2,cx,fy+sy*w/2,z)
        bolts+=[cylx(4.25,5,fx+sx*(w+2.5),cy,z)+cylx(2.5,w+4.4,fx+sx*(w-4.4)/2,cy,z),
                cyly(4.25,5,cx,fy+sy*(w+2.5),z)+cyly(2.5,w+4.4,cx,fy+sy*(w-4.4)/2,z),
                box(2.3,9.6,10,fx-sx*3.25,cy,z)-cylx(2.5,4,fx-sx*3.25,cy,z),box(9.6,2.3,10,cx,fy-sy*3.25,z)-cyly(2.5,4,cx,fy-sy*3.25,z)]
    # base-plate bolts: head in a 10 mm counterbore from the top, nut under the base plate
    base=[];pk=10
    for px,py in pedestal_base_bolts(d,x,y):
        body=body-cylz(2.75,z0-base_top+2,px,py,base_top-1)-cylz(5,pk+1,px,py,z0-pk)
        L=round((z0-pk)-(base_bottom-4)+2)
        base+=(screw(px,py,z0-pk,5,L)+nut(px,py,base_bottom-4,8,4,5)).solids()
    # scanner: 2x M3 from below, heads in pockets open at the overhanging bottom of the pedestal
    pts,_=scanner_screws(x,y,yaw,z0,deck,q.get('scanner_holes_local',[]))
    m3=[]
    for px,py in pts:
        body=body-cylz(1.7,deck+2,px,py,z0-deck-1)-cylz(3.5,z0-deck-base_top+1,px,py,base_top-1)
        m3+=screw(px,py,z0-deck,3,deck,True).solids()
    return [PartRecord('60_lidar_pedestal_PRINTED_'+n,'lidars',body,(.80,.66,.40),'concept_support',
                       {'material':PRINT_NOTE+'; print the walls with 6+ perimeters and 60% infill',
                        'design':f'Solid pedestal {z0-base_top:g} mm deep under the scanner + {w:g} mm walls {up:g} mm up two column faces',
                        'stiffness':'Hand estimate: 43 mm deep printed section (E~1.2 GPa effective) about 11 kN/mm at the scanner, first mode ~1 kHz; the 3t steel cantilever it replaces was about 0.9 kN/mm, ~300 Hz'}),
            PartRecord('65_lidar_column_bolts_M5_Tnut_'+n,'lidars',Compound(bolts),(.20,.22,.25),'catalog_envelope',
                       {'sku':'M5x10 hex socket + washer + M5 T-nut for 2020 slot 6','quantity':len(zs)*2}),
            PartRecord('65_lidar_base_bolts_M5_'+n,'lidars',Compound(base),(.20,.22,.25),'catalog_envelope',
                       {'sku':'M5x40 hex socket + washer + M5 nut under the base plate','quantity':len(pedestal_base_bolts(d,x,y))}),
            PartRecord('66_lidar_scanner_screws_M3_'+n,'lidars',Compound(m3),(.20,.22,.25),'catalog_envelope',
                       {'sku':'M3x8 + washer, 0.8 N m max (SICK)','quantity':len(pts),
                        'note':'Into the TiM571 bottom threads (M3 x 2.8 deep); hole spacing assumed, confirm on the SICK drawing'})]


def scanner_screws(x,y,yaw,plate_top,pt,holes):
    """TiM571 to its plate: M3 from below into the scanner's 2.8 mm deep bottom threads (SICK datasheet, max 0.8 N m).
    Hole positions are assumed (the SICK drawing spacing was not readable); the shank is drawn up to the scanner face."""
    from .suspension import screw
    pts=[]
    for lx,ly in holes:
        v=rz(box(1,1,1,lx,ly,0),yaw).translate((x,y,0)).bounding_box().center();pts.append((v.X,v.Y))
    return pts,Compound([s for px,py in pts for s in screw(px,py,plate_top-pt,3,pt,True).solids()])

def lidars(d):
    r=[];lidar=d.parts['lidar'];mount=d.layout['lidar_mount']
    for pose in d.layout['lidars']:
        n=pose['name'];x=pose['x'];y=pose['y'];z0=pose['bbox_bottom_z'];pt=mount['plate_thickness'];yaw=-pose['heading_deg']
        place=lambda obj:rz(obj,yaw).translate((x,y,0))
        sensorplate=place(box(mount['plate_width'],mount['plate_depth'],pt,0,0,z0-pt/2))
        if d.variant=='baseline':
            sx=pose['support_x'];sy=pose['support_y'];length=math.hypot(x-sx,y-sy)+25
            bridge=rz(box(length,25,pt,0,0,z0-pt/2),math.degrees(math.atan2(y-sy,x-sx))).translate(((x+sx)/2,(y+sy)/2,0))
            r.append(PartRecord('60_diagonal_lidar_mount_CONCEPT_'+n,'lidars',sensorplate+bridge,(.28,.43,.54)))
            r.append(column(d,sx,sy,d.p['base_bottom_z']+d.p['base_thickness'],z0-pt,'64_2020_Al_lidar_column_'+n,'lidars'))
        else:
            r.extend(lidar_pedestal(d,n,x,y,z0,yaw))
        r.append(PartRecord('61_SICK_TiM571_envelope_base_'+n,'lidars',place(box(lidar['width'],lidar['depth'],36,0,0,z0+18)),(.85,.88,.90),'drawing_envelope'))
        # Rev12b: the optical envelope stops under the 3mm scanner-top disc instead of overlapping it.
        top=3 if d.variant!='baseline' else 0
        r.append(PartRecord('62_SICK_TiM571_optical_envelope_'+n,'lidars',place(box(61,60,lidar['height']-36-top,0,0,z0+36+(lidar['height']-36-top)/2)),(.09,.16,.21),'drawing_envelope'))
        r.append(PartRecord('63_SICK_scanner_top_'+n,'lidars',cylz(22,3,x,y,z0+lidar['height']-3),(.20,.80,.79),'drawing_envelope'))
    return r

def camera_hanger(d,side,n,x,y,z,camera):
    """Rev17b: one 3D-printed hanger per camera (replaces the 2020 hanger column, the 3t adapter and the loose wedge).

    Top flange under the deck (2x M5 into heat-set inserts), 40x20 post, 6 mm platform, two ribs and the 10 deg / yaw
    wedge in one print."""
    from build123d import Location,Rotation
    from .suspension import screw,yz_prism
    from .electronics import PRINT_NOTE
    p=d.p;cm=d.layout['camera_mount'];hq=cm['hanger']
    deck_bottom=p['top_bottom_z'];deck_top=deck_bottom+p['top_thickness']
    top=cm['plate_center_z']+cm['plate_thickness']/2;py=cm['profile_y'];pw,pd=hq['post']
    plate=box(cm['plate_width'],cm['plate_depth'],hq['platform_mm'],x,cm['plate_y'],top-hq['platform_mm']/2)
    post=span(x-pw/2,x+pw/2,py-pd/2,py+pd/2,top,deck_bottom)
    flange=span(x-hq['flange'][0]/2,x+hq['flange'][0]/2,py-hq['flange'][1]/2,py+hq['flange'][1]/2,deck_bottom-hq['flange_mm'],deck_bottom)
    body=plate+post+flange
    for x0 in (x-pw/2,x+pw/2-5):
        body=body+yz_prism([(py+pd/2,top),(py+pd/2+hq['rib_depth_mm'],top),(py+pd/2,top+hq['rib_height_mm'])],x0,x0+5)
    frame=Location((x,y,z))*Rotation(0,0,-side*p['camera_yaw_deg'])*Rotation(-p['camera_pitch_down_deg'],0,0)
    local=camera.moved(frame.inverse()).bounding_box()
    block=box(local.size.X-4,local.size.Y-4,60,local.center().X,local.center().Y,local.min.Z-30).moved(frame)
    body=body+(block&box(cm['plate_width'],cm['plate_depth'],60,x,cm['plate_y'],top+30))
    body=body-camera   # ribs and wedge follow the yawed camera body exactly
    bolts=[]
    for bx in (x-hq['bolt_dx_mm'],x+hq['bolt_dx_mm']):
        body=body-cylz(3.2,hq['insert_depth_mm'],bx,py,deck_bottom-hq['insert_depth_mm'])
        bolts+=screw(bx,py,deck_top,5,p['top_thickness']+hq['insert_depth_mm']-1).solids()
    return [PartRecord('71_camera_hanger_PRINTED_'+n,'cameras',body,(.80,.66,.40),'concept_support',
                       {'material':PRINT_NOTE,'design':'Flange + 40x20 post + 6 mm platform + 2 ribs + wedge, one print; camera M2 screws pitch 80 not drawn',
                        'fit':'2x M5 heat-set inserts in the flange (D6.4 pockets)'}),
            PartRecord('73_camera_hanger_bolts_M5_'+n,'cameras',Compound(bolts),(.20,.22,.25),'catalog_envelope',
                       {'sku':'M5x12 + washer from the deck top into M5 heat-set inserts','quantity':2})]


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
        if d.variant!='baseline':
            r.extend(camera_hanger(d,side,n,x,y,z,r[-1].shape))
            continue
        adapter=box(cm['plate_width'],cm['plate_depth'],cm['plate_thickness'],x,cm['plate_y'],cm['plate_center_z'])
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
