
//--------------------------------
// PARAMETERS
//--------------------------------
$fn=16;

part = "assembly";

do_echo = true;

VER_MAJOR = 0;
VER_MINOR = 1;

di_08mm = 1.0;
eps = 0.01; // small number below printing tolerance

fuse_z = 50;  // fuselage height
theta = 6; // dihedral angle, deg (per wing, total is *2)
lambda = 0.5; // taper ratio
wing_loading = 3.45; // g/dm^2
m = 25; // flying mass, g,  with additional mocap payload
AR = 4.52; // aspect ratio, fune tuning to make longest arm be 300 mm
h_joint = 7; // joint length
wall = 0.6; // wall thickness
lg_theta = 45; // landing gear angle
deck_angle = 12; // tail dragger deck angle
lh = 290; // horizontal tail moment arm usually 2-3x cbar
lv = lh; // vertical tail moment arm (similar to lh)
aoi = 5; // angle of incidence with thrust axis

ARh = 3; // trainer 2-4, sport 3-5
ARv = 1.5;  // trainer 1-2, sport 1.5-2.5

Vh = 0.4; // horizontal tail volume coeff. (0.4-0.6 trainer), (0.3-0.5 sport)
Vv = 0.03; // vertical tail volume coeff. (0.03 - 0.05 trainer), (0.02-0.035 sport)

max_elev_deflect = 20; // elevator deflection (deg)

// TODO: auto calculate these using geometry
down_angle = 18.5;
down_angle_rear = 12.4;
theta_rear_fix = 0.18;
rudder_ratio = 0.3; // ratio of rudder to vertical tail area, trainer (0.25-0.4), sport (0.3-0.5)
elevator_ratio = 0.3; // ratio of elevator to horizontal tail area, trainer (0.25-0.4), sport (0.3-0.5)

fix = -lh - wall - .83 - di_08mm;

//--------------------------------
// CALCULATED VALUES
//--------------------------------

S = m/(wing_loading/1e4);  // wing area, g/dm^2
b = sqrt(AR*S); // wing span, mm (projected)
bp = b/cos(theta); // actual length of wing

c_root = 2*S/(b*(1 + lambda)); // chord at root, mm
c_tip = lambda*c_root; // chord at tip, mm
cbar = (c_root + c_tip)/2;
title = str("COGNI_UMX v", VER_MAJOR, ".", VER_MINOR);
alpha = atan2(c_root - c_tip, b/2);
Sh = Vh * S * cbar / lh; // horizontal tail area
Sv = Vv * S * b / lv;  // vertical tail area

hv = sqrt(ARv * Sv);  // height of vertical tail
bh = sqrt(ARh * Sh);  // span of horizontal tail

Sr = Sv * rudder_ratio; // area of rudder
Se = Sh * elevator_ratio; // area of elevator
ce = Se / bh; // chord of elevator
ch = 2 * Sh * (1 - elevator_ratio) / bh;  // root chord of horizontal tail (triangular)
cr = Sr / hv; // rectangular elevator
cv = 2 * Sv * (1 - rudder_ratio)/ hv;


assert((ch*bh/2 + ce*bh) == Sh, "horizontal tail area wrong");

if (do_echo) {
    echo(title);
    echo("wing area", S*1e-4, "dm^2");
    echo("wing span", b, "mm");
    echo("chord root", c_root, "mm");
    echo("chord tip", c_tip, "mm");
    echo("tail arm ratio", lh / cbar);
    echo("horizontal tail area", Sh*1e-4, "dm^2");
    echo("chord horz tail", ch, "mm");
    echo("check horz tail area", (ch*bh/2 + ce*bh)*1e-4, "dm^2");
    echo("chord vert tail", cv, "mm");
}

//--------------------------------
// MODULES
//--------------------------------

function deg2rad(d) = d * PI / 180;

function rotate_z(p, a) = [
    p[0]*cos(a) - p[1]*sin(a),
    p[0]*sin(a) + p[1]*cos(a),
    p[2]
];

function rotate_y(p, a) = [
    p[0]*cos(a) + p[2]*sin(a),
    p[1],
   -p[0]*sin(a) + p[2]*cos(a)
];

function rotate_x(p, a) = [
    p[0],
    p[1]*cos(a) - p[2]*sin(a),
    p[1]*sin(a) + p[2]*cos(a)
];

function rotate_euler_zyx(p, rot) = 
    rotate_z(
        rotate_y(
            rotate_x(p, rot[0]),  // X first
        rot[1]),                 // Y second
    rot[2]);                     // Z last


module multi_joint(h, azim, elev, d, through, wall, rounded, webbing) {
    // Creates a joint to connection multiple carbon fiber rods
    
    // given lists of equal length (h, azim, elev, d, through, wall)
    // h: length of each joint
    // azim: azimuth angles for each joint
    // elev: elevation angles of each joint
    // d: inner diameter for carbon fiber rods, if given list of 2, cuts a square
    // through: (whether or not hole goes all the way through)
    // wall: wall thickness
    // rounded: whether to use round or square joint, square better for printing
    // webbing [ [side1 , side2, x1, x2, len] , [...], ... ]  list of webbing triangles
    //     x1 and x2 define the offset distance from 0
    //     to define the thickness of the web
    difference() {
        multi_joint_solid(h, azim, elev, d, through, wall, rounded, webbing);
        multi_joint_drill_holes(h, azim, elev, d, through, wall, rounded);
    };
}

module multi_joint_solid(h, azim, elev, d, through, wall, rounded, webbing) {
    for (i = [0:len(h)-1]) {
        hi = h[i];
        r = eps;
        azimi = azim[i];
        elevi = elev[i];
        di = is_list(d[i]) ? max(d[i]) : d[i];
        walli = wall[i];
        roundedi = rounded[i];
        ro = walli + di/2;
        difference() {
            minkowski() {
                    rotate([0, 90 - elevi, azimi]) cylinder(h=hi, r=.0001);
                if (roundedi) {
                    sphere(r=ro);
                } else {
                    rotate([0, 90 - elevi, azimi]) cube(size=2*ro, center=true);
                }
            }
            // cut off end
            rotate([0, 90 - elevi, azimi])
            translate([0, 0, hi + ro])
            cube([4*ro, 4*ro, eps+2*ro], center=true);
        }
    }
    for (web = webbing) {
        j0 = web[0];
        j1 = web[1];
        x1 = web[2];
        x2 = web[3];
        h = web[4];

        p0 = rotate_euler_zyx([1.0, 0, 0], [0, -elev[j0], azim[j0]]);
        p1 = rotate_euler_zyx([1.0, 0, 0], [0, -elev[j1], azim[j1]]);
        cvect = cross(p0, p1);
        cuvect = cvect / norm(cvect);
        verts = [ for (j = [j0, j1])
            rotate_euler_zyx([h, 0, 0], [0, -elev[j], azim[j]]) + x1*cuvect
        ];
        verts2 = [ for (j = [j0, j1])
            rotate_euler_zyx([h, 0, 0], [0, -elev[j], azim[j]]) + x2*cuvect
        ];
        points = concat([x1*cuvect], verts, [x2*cuvect], verts2);
        A = 0;  // bottom center
        B = 1;
        C = 2;
        D = 3;  // top center
        E = 4;
        F = 5;
        faces = [
            [A, B, C],  // bottom triangle
            [D, F, E],  // top triangle (reversed winding for outward normal)

            // side faces (quads split into triangles)
            [A, D, E], [A, E, B],  // side 1
            [B, E, F], [B, F, C],  // side 2
            [C, F, D], [C, D, A]   // side 3
        ];
        polyhedron(points=points, faces=faces);
    }
}


module multi_joint_drill_holes(h, azim, elev, d, through, wall, rounded) {
    for (i = [0:len(h)-1]) { // cylinderical hole for each joint
        hi = h[i];
        r = 0.001;
        azimi = azim[i];
        elevi = elev[i];
        di = is_list(d[i]) ? max(d[i]) : d[i];
        walli = wall[i];
        throughi = through[i];
        roundedi = rounded[i];
        ro = walli + di/2;
        rotate([0, 90 - elevi, azimi]) union() {
            if (is_list(d[i])) {
                if (through[i]) {
                    translate([0, 0, 0]) cube([d[i][0],  d[i][1], 10*hi], center=true);
                } else {
                    translate([0, 0, 5*hi]) cube([d[i][0],  d[i][1], 10*hi], center=true);
                }
            } else {
                if (through[i]) {
                    translate([0, 0, -5*hi]) cylinder(h=10*hi, d=di); // cut tube for rod
                } else {
                    cylinder(h=10*hi, d=di); // cut tube for rod
                }
            }
        }
    }
}

module joint_wing_top_front() {
    multi_joint(
        h=[h_joint, h_joint, h_joint, h_joint],
        azim=[90, -90, 0, 0],
        elev=[theta, theta, -90, 0],
        d=[di_08mm, di_08mm, di_08mm, di_08mm],
        through=[false, false, false, false],
        wall=[wall, wall, wall, wall],
        rounded=[true, true, true, true],
        webbing=[
            [0, 2, di_08mm/2, di_08mm/2 + wall, h_joint],
            [0, 3, -wall/2, wall/2, h_joint],
            [3, 1, -wall/2, wall/2, h_joint],
            [3, 2, -wall/2, wall/2, h_joint],
            [2, 1, di_08mm/2, di_08mm/2 + wall, h_joint]
        ]);
}

module joint_wing_top_rear() {
    module solid() {
        multi_joint_solid(
            h=[h_joint, h_joint, h_joint, h_joint],
            azim=[90 - alpha, -90 + alpha, 0, 0],
            elev=[theta, theta, -90, 0],
            d=[di_08mm, di_08mm, di_08mm, di_08mm],
            through=[false, false, false, false],
            wall=[wall, wall, wall*1.2, wall],
            rounded=[true, true, true, true],
            webbing=[
            [0, 2, di_08mm/2, di_08mm/2 + 10*wall, h_joint],
            [0, 3, -wall/2, wall/2, h_joint],
            [3, 1, -wall/2, wall/2, h_joint],
            [3, 2, -wall/2, wall/2, h_joint],
            [2, 1, di_08mm/2, di_08mm/2 + 10*wall, h_joint]
            ]);
    }
    module holes() {
        multi_joint_drill_holes(
            h=[h_joint, h_joint, h_joint, h_joint],
            azim=[90 - alpha, -90 + alpha, 0, 0],
            elev=[theta, theta, -90, 0],
            d=[di_08mm, di_08mm, di_08mm, di_08mm],
            through=[false, false, false, false],
            wall=[wall, wall, wall, wall],
            rounded=[true, true, true, true]
        );
    }
    delta = di_08mm/2 + wall;
    difference() {
        solid();
        holes();
        translate([-100/2-delta, 0, 0]) cube([100, 100, 100], center=true);
    }
}

module joint_wing_bottom_front() {
    multi_joint(
        h=[h_joint, h_joint, h_joint, h_joint, h_joint, h_joint],
        azim=[90, -90, 0, 0, 90, -90],
        elev=[25, 25, 90, aoi, -lg_theta, -lg_theta],
        d=[di_08mm, di_08mm, di_08mm, [1.5, 1.5], di_08mm, di_08mm],
        through=[false, false, false, true, false, false],
        wall=[wall, wall, wall, wall, wall, wall],
        rounded=[true, true, true, false, true, true],
        webbing = [
            [1, 2, di_08mm/2, 1.5, h_joint],
            [3, 2, -wall/2, wall/2, h_joint],
            [3, 4, -wall/2, wall/2, h_joint],
            [3, 5, -wall/2, wall/2, h_joint],
            [4, 5, di_08mm/2, 1.5, h_joint],
            [5, 1, di_08mm/2, 1.5, h_joint],
            [2, 0, di_08mm/2, 1.5, h_joint],
            [0, 4, di_08mm/2, 1.5, h_joint]
        ]);
}

module joint_wing_bottom_rear() {
    module solid() {
        multi_joint_solid(
                h=[h_joint, h_joint, h_joint, h_joint],
            azim=[90 - alpha, -90 + alpha, 0, 0],
            elev=[19, 19, 90, -aoi],
            d=[di_08mm, di_08mm, di_08mm, [1.5, 1.5]],
            through=[false, false, false, true],
            wall=[wall, wall, 1.2*wall, wall],
            rounded=[true, true, true, false],
            webbing=[
                [2, 0, di_08mm/2, di_08mm/2 + wall*10, h_joint],
                [0, 3, -wall/2, wall/2, h_joint],
                [3, 1, -wall/2, wall/2, h_joint],
                [3, 2, -wall/2, wall/2, h_joint],
                [1, 2, di_08mm/2, di_08mm/2 + wall*10, h_joint],
            ]
        );
    }
    module holes() {
        multi_joint_drill_holes(
                h=[h_joint, h_joint, h_joint, h_joint],
            azim=[90 - alpha, -90 + alpha, 0, 0],
            elev=[19, 19, 90, -aoi],
            d=[di_08mm, di_08mm, di_08mm, [1.5, 1.5]],
            through=[false, false, false, true],
            wall=[wall, wall, wall, wall],
            rounded=[true, true, true, false]
        );
    }
    delta = di_08mm/2 + wall;
    difference() {
        solid();
        holes();
        translate([-100/2-delta, 0, 0]) cube([100, 100, 100], center=true);
    }
}

module joint_wing_top_front_right() {
    multi_joint(
        h=[h_joint, h_joint, h_joint],
        azim=[90, 90, 180],
        elev=[-down_angle, 0, 0],
        d=[di_08mm, di_08mm, di_08mm],
        through=[false, true, false],
        wall=[wall, wall, wall],
        rounded=[true, true, true],
        webbing = [
                [1, 2, di_08mm/2, di_08mm/2 + wall, h_joint],
                [0, 1, di_08mm/2, di_08mm/2 + wall, h_joint],
        ]
    );
}

module joint_wing_top_front_left() {
    mirror([0, 1, 0]) joint_wing_top_front_right();
}

module joint_wing_top_rear_right() {
    multi_joint(
        h=[h_joint, h_joint, h_joint],
        azim=[90+alpha, 90+alpha, 0],
        elev=[-18, 0, 0],
        d=[di_08mm, di_08mm, di_08mm],
        through=[true, false, false],
        wall=[wall, wall, wall],
        rounded=[true, true, true],
        webbing = [
        [2, 1, di_08mm/2, di_08mm/2 + wall, h_joint],
        [1, 0, di_08mm/2, di_08mm/2 + wall, h_joint],
    ]);
}

module joint_wing_top_rear_left() {
    mirror([0, 1, 0]) joint_wing_top_rear_right();
}

module joint_wing_top_front_right_tip() {
    multi_joint(
        h=[h_joint, h_joint],
        azim=[90, 180],
        elev=[0, 0],
        d=[di_08mm, di_08mm],
        through=[false, false],
        wall=[wall, wall],
        rounded=[true, true, true],
        webbing = [
            [0, 1, di_08mm/2, di_08mm/2 + wall, h_joint]
        ]);
}

module joint_wing_top_front_left_tip() {
    mirror([0, 1, 0]) joint_wing_top_front_right_tip();
}

module joint_wing_top_rear_right_tip() {
    multi_joint(
        h=[h_joint, h_joint],
        azim=[0, 90 + alpha],
        elev=[0, 0],
        d=[di_08mm, di_08mm],
        through=[false, false],
        wall=[wall, wall],
        rounded=[true, true, true],
        webbing = [
            [0, 1, di_08mm/2, di_08mm/2 + wall, h_joint]
        ]);
}

module joint_wing_top_rear_left_tip() {
    mirror([0, 1, 0]) joint_wing_top_rear_right_tip();
}

module joint_gear_elevator() {
    x1 = -wall - 1.5/2;
    x2 = x1 - di_08mm/2;
    x3 = x2 - 1.5*di_08mm - wall/2;
    delta = (1.5 - di_08mm)/2;
    
    difference() {
        union() {
            translate([1, 0, 0]) cube([h_joint*1.8, 3, 3], true);
            multi_joint_solid(
                h=[h_joint],
                azim=[0],
                elev=[0],
                d=[[1.5, 1.5]],
                through=[false],
                wall=[wall],
                rounded=[false]);
            translate([0, 0.25, 1.5/2 + eps]) rotate([90, 0, 0]) linear_extrude(0.5)
                polygon([[x3 + di_08mm/2 + wall,0],[h_joint,0],[x3 + di_08mm/2,h_joint*0.7]]);
            
            translate([0, -0.25, -1.5/2 - eps]) rotate([270, 0, 0]) linear_extrude(0.5)
                polygon([[x3 + di_08mm/2 + wall,0],[h_joint,0],[x3 + di_08mm/2,h_joint*0.7]]);
            
            translate([x2, 0, 0]) multi_joint_solid(
                h=[h_joint/2, h_joint/2],
                azim=[90, -90],
                elev=[0, 0],
                d=[di_08mm, di_08mm],
                through=[true],
                wall=[wall+delta, wall+delta],
                rounded=[true, true]);
            
            translate([x3, 0, -delta- eps]) multi_joint_solid(
                h=[h_joint, h_joint],
                azim=[0, 0],
                elev=[90, 270],
                d=[di_08mm, di_08mm],
                through=[true, false],
                wall=[wall, wall],
                rounded=[true, true]);
        }
        multi_joint_drill_holes(
            h=[h_joint],
            azim=[0],
            elev=[0],
            d=[[1.5, 1.5]],
            through=[false],
            wall=[wall],
            rounded=[false]);
        translate([x2, 0, 0]) multi_joint_drill_holes(
            h=[h_joint/2, h_joint/2],
            azim=[90, -90],
            elev=[0, 0],
            d=[di_08mm, di_08mm],
            through=[true],
            wall=[wall+delta, wall+delta],
            rounded=[true, true]);
        translate([x3, 0, -delta- eps]) multi_joint_drill_holes(
            h=[h_joint],
            azim=[0],
            elev=[90],
            d=[di_08mm],
            through=[true],
            wall=[wall],
            rounded=[true]);
    }
    

}

module cf_rod(rod) {
    rotate([0, 90, 0]) color("black") cylinder(h=rod[0], d=rod[1]);
}

module cf_square_rod(rod) {
    translate([-rod[0]/2, 0, 0]) rotate([0, 90, 0]) color("black")
    cube([rod[1][0], rod[1][1], rod[0]], center=true);
}

module steel_rod(rod) {
    rotate([0, 90, 0]) color("grey") cylinder(h=rod[0], d=rod[1]);
}

module wheel(d, spokes=8, thick=1, d_axle_cut=di_08mm) {
    rotate([90, 0, 0]) union() {
        rotate_extrude(convexity = 20, $fn=100)
            translate([d/2, 0, 0])
                square(size = thick, center=true);
        rotate([0, -90, 0]) translate([wall, 0, 0]) multi_joint(
            h=[h_joint],
            azim=[0],
            elev=[0],
            d=[d_axle_cut],
            through=[true],
            wall=[wall],
            rounded=true);
        difference() {
            for (i = [0:spokes]) {
                rotate([360*i/spokes, 90, 0]) translate([0, 0, d/4]) cube([thick, thick, d/2], center=true);
            }
            cylinder(h=h_joint, d=d_axle_cut, center=true);
        }
    }
}

module airfoil(chord, width) {
    resolution = 30;
    camber = 0.08;
    polygon([ for (i = [-resolution : resolution - 1])
        let (
            x = chord * abs(i) / resolution,
            xc = 2 * x / chord - 1,
            y = camber * chord * sqrt(max(0, 1 - xc * xc))
        ) [x - chord, if (i > 0)  y else y - eps]
    ]);
}

module airfoil_elliptical(chord, span, lambda, width, thickness) {
    translate([0, 0, 2]) difference() {
        rotate([90-theta, 0, 0])
        minkowski() {
            linear_extrude(width/cos(theta), scale=1 - lambda*width/(span/2))
                projection(false) rotate([theta, 0, 0])
                linear_extrude(eps) airfoil(chord);
            cube(size=thickness, center=true);
        }
        translate([-chord, 0, 0]) cube([2*chord, 2*chord, 2*chord]);
        translate([-chord, -width*cos(theta)-2*chord, 0]) cube([2*chord, 2*chord, 2*chord]);
    }
}

module joint_right_ang() {
    multi_joint(
        h=[h_joint, h_joint],
        azim=[90, 180],
        elev=[0, 0],
        d=[di_08mm, di_08mm],
        through=[false, false],
        wall=[wall, wall],
        rounded=[true, true, true],
        webbing = [
            [1, 0, di_08mm/2, di_08mm/2 + wall, h_joint]
        ]);
}

module joint_front_rudder() {
    multi_joint(
        h=[h_joint, h_joint, h_joint],
        azim=[90, 180-max_elev_deflect, 270],
        elev=[0, 0, 0],
        d=[di_08mm, di_08mm, di_08mm],
        through=[false, false, true],
        wall=[wall, wall, wall],
        rounded=[true, true, true],
        webbing = [
            [2, 1, di_08mm/2, di_08mm/2 + wall, h_joint],
            [1, 0, di_08mm/2, di_08mm/2 + wall, h_joint],
        ]);
}

module joint_back_rudder() {
    multi_joint(
        h=[h_joint, h_joint],
        azim=[90, 180+max_elev_deflect],
        elev=[0, 0],
        d=[di_08mm, di_08mm],
        through=[false, false],
        wall=[wall, wall],
        rounded=[true, true],
        webbing = [
            [1, 0, di_08mm/2, di_08mm/2 + wall, h_joint],
        ]);
}

module servo_arm(
        Wb = 6,
        Wt = 4,
        L = 20,
        thickness = 2,
        segments = 10,
        hole_count = 3,
        hole_diameter = 1,
        hole_spacing = 3,
        hole_base_offset = 12,
        base_cyl_diameter = di_08mm + 2*wall,
        base_cyl_length = h_joint,
        base_cyl_hole_diameter = di_08mm)
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


//--------------------------------
// TAIL
//--------------------------------

module Tail() {
    x1 = -wall - 1.5/2;
    x2 = x1 - di_08mm/2;
    x3 = x2 - 1.5*di_08mm - wall/2;
    
    // make sure to change cf_rods list to match these values
    
    // Sh = Vh*S*c / lh
    // ARt = bt/ct
    // Sh = bt * ct  
    
   
    
    
    
    S = AR * ((c_tip + c_root) / 2)^2;
    Sh = Vh* S *((c_tip + c_root) / 2) / lh;
    cht = sqrt(Sh / ARh);
    bht = ARh * cht;
 
    Sv = Vv* S * AR * ((c_tip + c_root) / 2) / lv;
    cvt = sqrt(Sv / ARv);
    bvt = ARv * cvt;
    cvt_tip = cvt * bvt / (bvt - 2*h_joint / 3); // accounting for the triangluar shape at the nottom of the rudder
    
    
    // vert tail
    translate([-x3, 0, 0]) rotate([0, 0, 0]) joint_gear_elevator();
    translate([0, 0, 2*h_joint]) rotate([90, 0, 0]) joint_front_rudder();
    translate([0, 0, bvt]) rotate([-90, 0, 0]) joint_right_ang();
    translate([-cvt_tip, 0, bvt]) rotate([-90, 0, 180]) joint_right_ang();
    translate([-cvt_tip, 0, 2*h_joint + cvt_tip*tan(max_elev_deflect)]) rotate([90, 0, 180]) joint_back_rudder();
    
    translate([0,0,bvt]) rotate([0,90,0]) cf_rod([bvt, 0.8]);
    translate([-cvt_tip,0,bvt]) rotate([0,90,0]) cf_rod([bvt - cvt_tip*tan(max_elev_deflect) - 14, 0.8]);
    translate([-cvt_tip,0,bvt]) rotate([0,0,0]) cf_rod([cvt_tip, 0.8]);
    translate([-cvt_tip,0,2*h_joint + cvt_tip*tan(max_elev_deflect)]) rotate([180,max_elev_deflect,0]) cf_rod([cvt_tip / cos(max_elev_deflect), 0.8]);
    
    translate([0,0,3*h_joint]) rotate([180,180,0]) servo_arm();
    
    
    // horizontal tail
    translate([-x2, bht / 2, 0]) rotate([180, 0, 0]) joint_right_ang();
    translate([-x2, -bht / 2, 0]) rotate([0, 0, 0]) joint_right_ang();
    translate([-x2 - cht, bht / 2, 0]) rotate([180, 180, 0]) joint_right_ang();
    translate([-x2 - cht, -bht / 2, 0]) rotate([0, 0, 270]) joint_right_ang();
    
    translate([-x2,-bht / 2,0]) rotate([0,0,90]) cf_rod([bht, 0.8]);
    translate([-x2 - cht,-bht / 2,0]) rotate([0,0,90]) cf_rod([bht, 0.8]);
    translate([-x2 - cht,-bht / 2,0]) rotate([0,0,0]) cf_rod([cht, 0.8]);
    translate([-x2 - cht, bht / 2,0]) rotate([0,0,0]) cf_rod([cht, 0.8]);
    
    translate([-x2, 4, 0]) rotate([270,180,0]) servo_arm();
    
}


//--------------------------------
// RODS
//--------------------------------

cf_rods = [
    // length, diameter (mm)
    [c_root, 0.8], // 0
    [sqrt((bp/2)^2 + (c_root - c_tip)^2), 0.8], // 1
    [sqrt((bp/2)^2 + (c_root - c_tip)^2), 0.8], // 2
    [bp/2, 0.8], // 3
    [bp/2, 0.8], // 4
    [fuse_z- c_root*tan(aoi), 0.8], // 5
    [fuse_z, 0.8], // 6
    [157, 0.8], // 7
    [157, 0.8], // 8
    [c_root*(1 - lambda*0.5), 0.8], //9
    [c_root*(1 - lambda*0.5), 0.8], //10
    [c_tip, 0.8], //11
    [c_tip, 0.8], //12
    [bh, 0.8], // 13
    
    // 14 through 17 all pertain to tail, make sure they match the parameters in the tail module
    [100, 0.8], // 14 tail height (z)
    [86, 0.8], // 15 rudder height (z) (must be at least 14 less than overall height)
    [65, 0.8], // 16 tail length (x)
    [300, 0.8], // 17 tail width (y)
];

steel_rods = [
    // length, diameter (mm)
    [70, 1],
    [70, 1],
];

cf_square_rods = [
    // length, [width, height]
    [300, [1.5, 1.5]],
];


module rods() {
    translate([-c_root, 0, 0]) cf_rod(cf_rods[0]);
    translate([-c_root, 0, 0]) rotate([0, -theta + theta_rear_fix, 90-alpha]) cf_rod(cf_rods[1]);
    translate([-c_root, 0, 0]) rotate([0, -theta + theta_rear_fix, -90+alpha]) cf_rod(cf_rods[2]);
    translate([0, 0, 0]) rotate([0, -theta, 90]) cf_rod(cf_rods[3]);
    translate([0, 0, 0]) rotate([0, -theta, -90]) cf_rod(cf_rods[4]);
    translate([-c_root, 0, -fuse_z + c_root*tan(aoi)]) rotate([0, -90, 0]) cf_rod(cf_rods[5]);
    translate([0, 0, -fuse_z]) rotate([0, -90, 0]) cf_rod(cf_rods[6]);

    translate([0,  0, -fuse_z]) rotate([0, lg_theta, 90])  steel_rod(steel_rods[0]);
    translate([0,  0, -fuse_z]) rotate([0, lg_theta, -90])  steel_rod(steel_rods[1]);

    rotate([0, aoi, 0]) translate([10, 0, -fuse_z]) cf_square_rod(cf_square_rods[0]);

    translate([0, 50, -fuse_z-50]) rotate([0, 0, 180]) wheel(d=40);
    translate([0, -50, -fuse_z-50]) wheel(d=40);
  
    translate([-lh, 0, -fuse_z]) wheel(d=30);

    translate([0, 0, -fuse_z]) rotate([0, -theta -down_angle, 90]) cf_rod(cf_rods[7]);
    translate([0, 0, -fuse_z]) rotate([0, -theta -down_angle, -90]) cf_rod(cf_rods[8]);

    translate([-c_root, 0, -fuse_z + c_root*tan(aoi)]) rotate([0, -theta -down_angle_rear, 90-alpha]) cf_rod(cf_rods[7]);
    translate([-c_root, 0, -fuse_z + c_root*tan(aoi)]) rotate([0, -theta -down_angle_rear, -90+alpha]) cf_rod(cf_rods[8]);
 
    rotate([theta, 0, 180]) translate([0, bp/4, 0]) cf_rod(cf_rods[9]);
    rotate([-theta, 0, 180]) translate([0, -bp/4, 0]) cf_rod(cf_rods[10]);
    
    rotate([theta, 0, 180]) translate([0, bp/2, 0]) cf_rod(cf_rods[11]);
    rotate([-theta, 0, 180]) translate([0, -bp/2, 0]) cf_rod(cf_rods[12]);
}

module joints() {
    rotate([0, 0, 180]) translate([0, 0, -fuse_z]) joint_wing_bottom_front();
    translate([-c_root, 0, -fuse_z + c_root*tan(aoi)]) joint_wing_bottom_rear();
    
    rotate([theta, 0, 0]) translate([0, bp/4, 0]) joint_wing_top_front_left();
    rotate([-theta, 0, 0]) translate([0, -bp/4, 0]) joint_wing_top_front_right();

    rotate([theta, 0, 0]) translate([0, bp/2, 0]) joint_wing_top_front_left_tip();
    rotate([-theta, 0, 0]) translate([0, -bp/2, 0]) joint_wing_top_front_right_tip();
    
    rotate([theta, 0, 0]) translate([-c_root*(1 - lambda*0.5), bp/4, 0]) joint_wing_top_rear_left();
    rotate([-theta, 0, 0]) translate([-c_root*(1 - lambda*0.5), -bp/4, 0]) joint_wing_top_rear_right();
    
    rotate([theta, 0, 0]) translate([-c_tip, bp/2, 0]) joint_wing_top_rear_left_tip();
    rotate([-theta, 0, 0]) translate([-c_tip, -bp/2, 0]) joint_wing_top_rear_right_tip();

    translate([0, 0, 0]) rotate([0, 0, 180]) joint_wing_top_front();
    translate([-c_root, 0, 0]) rotate([0, 0, 0]) joint_wing_top_rear();
    
    x1 = -wall - 1.5/2;
    x2 = x1 - di_08mm/2;
    x3 = x2 - 1.5*di_08mm - wall/2;
    
    translate([-lh + x3, 0, -fuse_z + lh*tan(aoi)]) rotate([0, aoi, 0]) Tail();
    
    
 
}

module assembly() {
    rods();
    joints();
    //airfoil_elliptical(chord=c_root, camber=0.08, resolution=30);
}

module rod_template() {
    // this module creates a printable rod template and also echos rod lengths
    n_cf = len(cf_rods);
    n_cf_sq = len(cf_square_rods);
    n_steel = len(steel_rods);
    n = n_cf + n_cf_sq + n_steel;
    thick = 3;
    space = 10;
    difference() {
        linear_extrude(thick) square([305, (n+1)*space]);
        translate([180, -eps, -thick]) linear_extrude(3*thick) square([150, 110]);
        translate([150, -80, -thick]) linear_extrude(3*thick) rotate([0, 0, 45]) square([155, 113]);
        translate([5, n*space + 3, thick + eps - 1])
                linear_extrude(1) text(title, size=5);
        for (i = [0:n_steel-1]) {
            rod = steel_rods[i];
            translate([-eps, space*(i) + 10, thick + eps -rod[1]])
                linear_extrude(rod[1]) square([rod[0], rod[1]]);
            label = str("steel", i, " l:", round(rod[0]), " r:", rod[1]);
            echo(label);
            translate([5, space*(i) + 3, thick + eps - 1])
                linear_extrude(1) text(label, size=5);
        }
        cf_rods_odered = [5, 6, 11, 12, 9, 10, 7, 8, 0, 3, 4, 1, 2];
        for (i = [0:n_cf-1]) {
            j = cf_rods_odered[i];
            rod = cf_rods[j];
            translate([-eps, space*(i + n_steel) + 10, thick + eps -rod[1]])
                linear_extrude(rod[1]) square([rod[0], rod[1]]);
            label = str("cf", j, " l:", round(rod[0]), " r:", rod[1]);
            echo(label);
            translate([5, space*(i + n_steel) + 3, thick + eps - 1]) linear_extrude(1) text(label, size=5);
        }
        for (i = [0:n_cf_sq-1]) {
            rod = cf_square_rods[i];
            translate([-eps, space*(i + n_steel + n_cf) + 10, thick + eps -rod[1][1]])
                linear_extrude(rod[1][1]) square([rod[0], rod[1][0]]);
            label = str("cf_sq", i, " l:", round(rod[0]), " d:", rod[1]);
            echo(label);
            translate([5, space*(i + n_steel + n_cf) + 3, thick + eps - 1])
                linear_extrude(1) text(label, size=5);
        }

    }
}

//--------------------------------
// Tail Joints Print Layout
//--------------------------------
module Tail_print() {
    
    x = di_08mm / 2 + wall;
    
    translate([15, 5, h_joint*1.8 / 2 - 1]) rotate([0, 270, 0]) joint_gear_elevator();
    translate([15, 20, x]) rotate([0, 0, 0]) joint_T();
    translate([15, 35, x]) rotate([0, 0, 0]) joint_right_ang();
    translate([32, 5, x]) rotate([0, 0, 0]) joint_right_ang();
    translate([32, 20, x]) rotate([0, 0, 0]) joint_right_ang(); 
    translate([32, 35, x]) rotate([0, 0, 0]) joint_right_ang();
    translate([45, 5, x]) rotate([0, 0, 0]) joint_right_ang();
    translate([45, 20, x]) rotate([0, 0, 0]) joint_right_ang();
    translate([45, 35, x]) rotate([0, 0, 0]) joint_right_ang();
}

//--------------------------------
// PART OUTPUT
//--------------------------------
if (part == "joint_wing_top_front") {
    joint_wing_top_front();
} else if (part == "joint_wing_top_rear") {
    joint_wing_top_rear();
} else if (part == "joint_wing_top_front_left") {
    joint_wing_top_front_left();
} else if (part == "joint_wing_top_front_right") {
    joint_wing_top_front_right();
} else if (part == "joint_wing_top_front_left_tip") {
    joint_wing_top_front_left_tip();
} else if (part == "joint_wing_top_front_right_tip") {
    joint_wing_top_front_right_tip();
} else if (part == "joint_wing_top_rear_left") {
    joint_wing_top_rear_left();
} else if (part == "joint_wing_top_rear_right") {
    joint_wing_top_rear_right();
} else if (part == "joint_wing_top_rear_left_tip") {
    joint_wing_top_rear_left_tip();
} else if (part == "joint_wing_top_rear_right_tip") {
    joint_wing_top_rear_right_tip();
} else if (part == "joint_wing_bottom_front") {
    joint_wing_bottom_front();
} else if (part == "joint_wing_bottom_rear") {
    joint_wing_bottom_rear();
} else if (part == "joint_gear_elevator") {
    joint_gear_elevator();
} else if (part == "joint_right_ang") {
  joint_right_ang();  
} else if (part == "joint_T") {
  joint_T(); 
} else if (part == "rod_template") {
    rod_template();
} else if (part == "tail") {
    rotate([0, 0, 0]) Tail();
 } else if (part == "tail_print") {
    rotate([0, 0, 0]) Tail_print();
} else if (part == "assembly") {
    rotate([0, -aoi, 0]) assembly();
} else {
    assert(false, str("unkonwn part: ", part));
}