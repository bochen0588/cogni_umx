module servo_arm(
        Wb, Wt, L, thickness, segments,
        hole_count, hole_diameter, hole_spacing, hole_base_offset,
        base_cyl_diameter, base_cyl_length, base_cyl_hole_diameter)
/* 
Creates a servo arm with through holes and a base connection for a
carbon fiber rod

Wb : base width
Wt : width at start of circular arc
L  : overall lenght 
thickness : Z‑extrusion depth
segments : number of segments in tip arc (Changes shape of tip)

hole_count : number of holes in arm
hole_diameter : diameter of holes in arm
hole_spacing : spacing between holes in arm
hole_base_offset : offset of holes from the base of the arm

base_cyl_diameter : diameter of the connector
base_cyl_length : length of connector (length measured from top surface of the body, negative values inverse connector direction)
base_cyl_hole_diameter : diameter of hole in connector
*/
{
    // find height where arc begins by sampling
    steps = 200;
    candidates = [for(i=[0:steps]) i*L/steps];
    errors = [
        for(h=candidates)
            let(
                dx = abs(Wb-Wt)/2,
                t  = [dx,h]/norm([dx,h]),               // unit tangent
                n  = [ t[1], -t[0] ],                   // outward normal
                P1 = [-Wt/2, h],
                P2 = [ Wt/2, h],
                b  = [P2[0]-P1[0], P2[1]-P1[1]],
                n2 = [-n[0], n[1]],
                det = n[0]*(-n2[1]) - (-n2[0])*n[1],
                s   = det==0 ? 0 :
                      ( b[0]*(-n2[1]) - b[1]*(-n2[0]) )/det,
                C   = [P1[0] + s*n[0], P1[1] + s*n[1]],   // center
                R   = norm([C[0]-P1[0], C[1]-P1[1]]),
                top = C[1] + R                            // arc apex y
            ) abs(top - L)
    ];
    H = candidates[search(min(errors), errors)[0]];

    // recompute circle data with the chosen H
    dx = abs(Wb-Wt)/2;
    t  = [dx,H]/norm([dx,H]);
    n  = [ t[1], -t[0] ];
    P1 = [-Wt/2, H];
    P2 = [ Wt/2, H];
    b  = [P2[0]-P1[0], P2[1]-P1[1]];
    n2 = [-n[0], n[1]];
    det = n[0]*(-n2[1]) - (-n2[0])*n[1];
    s   = det==0 ? 0 :
          ( b[0]*(-n2[1]) - b[1]*(-n2[0]) )/det;
    C   = [P1[0] + s*n[0], P1[1] + s*n[1]];
    R   = norm([C[0]-P1[0], C[1]-P1[1]]);

    // arc point list
    theta1 = atan2(P1[1]-C[1], P1[0]-C[0]);
    theta2 = atan2(P2[1]-C[1], P2[0]-C[0]);
    angles = [for(i=[0:segments]) theta1 + (theta2-theta1)*i/segments];
    arcpts = [for(a=angles) [C[0]+R*cos(a), C[1]+R*sin(a)]];

    // outline polygon
    shape = concat([[-Wb/2,0],[Wb/2,0],P2], arcpts, [P1]);

    // vertical‑hole coordinates
    vholes = [for(i=[0:hole_count-1]) [0, hole_base_offset + i*hole_spacing]];

    // 3‑D construction
    difference() {
        union() {
            // body 
            linear_extrude(height=thickness)
                polygon(points=shape);

            // optional base cylinder 
            if(base_cyl_diameter>0)
                translate([0,0,base_cyl_length/2 + thickness/2])
                    rotate([0,0,90])
                        cylinder(h=abs(base_cyl_length) + thickness,d=       base_cyl_diameter,center=true,$fn=60);
        }

        // vertical holes 
        for(p=vholes)
            translate([p[0],p[1],-1])
                cylinder(h=thickness+2,d=hole_diameter,$fn=30);

        // coaxial hole through connecting cylinder and base 
        if(base_cyl_diameter>0 && base_cyl_hole_diameter>0)
            translate([0,0,base_cyl_length/2 + thickness/2])
                rotate([0,0,90])
                    cylinder(h=abs(base_cyl_length)+thickness+10,
                             d=base_cyl_hole_diameter,
                             center=true,$fn=30);
    }
}

// Render
servo_arm(
    Wb = 10,
    Wt = 5,
    L = 60,
    thickness = 5,
    segments = 60,
    hole_count = 3,
    hole_diameter = 2,
    hole_spacing = 10,
    hole_base_offset = 30,
    base_cyl_diameter = 10,
    base_cyl_length = 15,
    base_cyl_hole_diameter = 5);