from .common import *

def make(d,side):
    wheel=d.parts['drive_motor'];n='LEFT' if side<0 else 'RIGHT';group='motor_left' if side<0 else 'motor_right'
    xc=side*d.p['drive_track']/2;z=wheel['tire_dia']/2
    outer=-wheel['tread_width']/2-wheel['outer_face_to_tire'];end=outer+wheel['overall_axial'];body=end-wheel['body_face_to_shaft_end']
    tire=cylx(wheel['tire_dia']/2,wheel['tread_width'],xc,0,z)-cylx(wheel['assumed_body_dia']/2,wheel['tread_width']+2,xc,0,z)
    casing=cylx(wheel['assumed_body_dia']/2,body-outer,xc-side*(outer+body)/2,0,z)
    axle=cylx(wheel['shaft_dia']/2,end-body,xc-side*(body+end)/2,0,z)&box(100,wheel['shaft_flat'],30,xc-side*45,0,z)
    records=[PartRecord('30_NBR_OD180_W42p5_'+n,group,tire,(.09,.13,.18),'datasheet_dimension'),
            PartRecord('31_casing_OD140_ASSUMED_axial_DATASHEET_'+n,group,casing,(.27,.48,.83),'assumed_casing'),
            PartRecord('32_D14_AF10_M14x1p5_THREAD_BLANK_'+n,group,axle,(.16,.27,.35),'datasheet_dimension')]
    if d.variant!='baseline':
        for item in records:
            item.metadata['moving_wheel']=n
            item.shape=item.shape.translate((0,0,d.motion.get(n,0)))
            item.__post_init__()
    return records
