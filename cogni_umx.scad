
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

// TODO: auto calculate these using geometry
down_angle = 18.5;
down_angle_rear = 12.4;
theta_rear_fix = 0.18;
rudder_ratio = 0.3; // ratio of rudder to vertical tail area, trainer (0.25-0.4), sport (0.3-0.5)
elevator_ratio = 0.3; // ratio of elevator to horizontal tail area, trainer (0.25-0.4), sport (0.3-0.5)

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


htail_start = lh - ch;
vtail_start = lh - cv;

htail_sweep = atan((bh/2)/ch);
vtail_sweep = atan(hv/cv);

htail_spar =  ch/cos(htail_sweep);
vtail_spar =  cv/cos(vtail_sweep);


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
    echo("htail sweep", htail_sweep, "deg");
    echo("vtail sweep", vtail_sweep, "deg");
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

module joint_vtail_front() {
    multi_joint(
        h=[h_joint, h_joint],
        azim=[180, 180],
        elev=[0, vtail_sweep],
        d=[[1.5, 1.5], di_08mm],
        through=[true, false],
        wall=[wall, wall],
        rounded=[false, true],
        webbing = [
            [0, 1, -wall/2, wall/2, h_joint]
        ]);
}

module joint_htail_front() {
    multi_joint(
        h=[h_joint, h_joint, h_joint],
        azim=[180, -(90 + htail_sweep), (90 + htail_sweep)],
        elev=[0, 0, 0],
        d=[[1.5, 1.5], di_08mm, di_08mm],
        through=[true, false, false],
        wall=[wall, wall, wall],
        rounded=[false, true, true],
        webbing = [
            [1, 0, di_08mm/2, di_08mm/2 + wall + 0.25, h_joint],
            [0, 2, di_08mm/2, di_08mm/2 + wall + 0.25, h_joint]
        ]);
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
            
            translate([x2, 0, 0]) multi_joint_solid(
                h=[h_joint/2, h_joint/2],
                azim=[90, -90],
                elev=[0, 0],
                d=[di_08mm, di_08mm],
                through=[true],
                wall=[wall+delta, wall+delta],
                rounded=[true, true]);
            
            translate([x3, 0, -delta- eps]) multi_joint_solid(
                h=[h_joint],
                azim=[0],
                elev=[90],
                d=[di_08mm],
                through=[true],
                wall=[wall],
                rounded=[true]);
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
    [htail_spar, 0.8], // 13
    [htail_spar, 0.8], // 14
    [vtail_spar, 0.8], // 15
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
    
    translate([-htail_start, 0, -fuse_z + htail_start*tan(aoi)])
    rotate([0, 0, 180 - htail_sweep])  cf_rod(cf_rods[13]); // htail spar 1

    translate([-htail_start, 0, -fuse_z + htail_start*tan(aoi)])
    rotate([0, 0, 180 + htail_sweep])  cf_rod(cf_rods[14]); // htail spar 2
   
    translate([-vtail_start, 0, -fuse_z + vtail_start*tan(aoi)])
    rotate([0, -vtail_sweep, 180])  cf_rod(cf_rods[15]); // htail spar
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
    
    translate([-htail_start, 0, -fuse_z + htail_start*tan(aoi)]) rotate([0, aoi, 0]) joint_htail_front();
    translate([-vtail_start, 0, -fuse_z + vtail_start*tan(aoi)]) rotate([0, aoi, 0]) joint_vtail_front();
    translate([-lh, 0, -fuse_z + lh*tan(aoi)]) rotate([0, aoi, 0]) joint_gear_elevator();
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
} else if (part == "rod_template") {
    rod_template();
} else if (part == "assembly") {
    rotate([0, -aoi, 0]) assembly();
} else {
    assert(false, str("unkonwn part: ", part));
}
