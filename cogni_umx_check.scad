//--------------------------------
// IMPORTS
//--------------------------------
include <BOSL2/std.scad>

//--------------------------------
// PARAMETERS
//--------------------------------
$fn=16;

part = "wing_mold_top";

do_echo = true;

VER_MAJOR = 0;
VER_MINOR = 1;

di_08mm = 1.0;
eps = 0.01; // small number below printing tolerance

fuse_z = 50;  // fuselage height
theta = 6; // dihedral angle, deg (per wing, total is *2)
lambda = 0.5; // taper ratio
wing_loading = 3.06; // g/dm^2
m = 34.2; // flying mass, g,  with additional mocap payload
AR = 4.52; // aspect ratio, fune tuning to make longest arm be 300 mm
h_joint = 7; // joint length
wall = 0.6; // wall thickness
lg_theta = 45; // landing gear angle
deck_angle = 12; // tail dragger deck angle
lh = 290; // horizontal tail moment arm usually 2-3x cbar
lv = lh; // vertical tail moment arm (similar to lh)
aoi = 5; // angle of incidence with thrust axis

ARh = 3; // trainer 2-4, sport 3-5
ARv = 1;  // trainer 1-2, sport 1.5-2.5

Vh = 0.3; // horizontal tail volume coeff. (0.4-0.6 trainer), (0.3-0.5 sport)
Vv = 0.05; // vertical tail volume coeff. (0.03 - 0.05 trainer), (0.02-0.035 sport)

max_elev_deflect = 25; // elevator deflection (deg)

// TODO: auto calculate these using geometry
down_angle = 18.5;
down_angle_rear = 12.4;
theta_rear_fix = 0.18;
rudder_ratio = 0.5; // ratio of rudder to vertical tail area, trainer (0.25-0.4), sport (0.3-0.5)
elevator_ratio = 0.49; // ratio of elevator to horizontal tail area, trainer (0.25-0.4), sport (0.3-0.5)

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
//cr_tip = cr; // * hv / (hv - cr_tip*tan(max_elev_deflect)/2); // accounting for the triangluar shape at the bottom of the rudder
cr_tip = (-hv - sqrt(hv^2 - 2*hv*cr*tan(max_elev_deflect)) / -tan(max_elev_deflect));


assert((ch*bh/2 + ce*bh) == Sh, "horizontal tail area wrong");

	back_top_main_wing = sqrt((bp/2)^2 + (c_root - c_tip)^2);
	front_top_main_wing = bp/2;
	back_middle_main_wing = fuse_z- c_root*tan(aoi);
	front_middle_main_wing = fuse_z;
	front_main_wing_support = 157;
	back_main_wing_support = 157;
	

if (do_echo) {
    echo(title);
    echo("wing area", S*1e-4, "dm^2");
    echo("wing span", b, "mm");
    echo("chord root", c_root, "mm");
    echo("chord tip", c_tip, "mm");
    echo("tail arm ratio", lh / cbar);
    echo("horizontal tail area", Sh*1e-4, "dm^2");
    echo("chord h orz tail", ch, "mm");
    echo("check horz tail area", (ch*bh/2 + ce*bh)*1e-4, "dm^2");
    echo("chord vert tail", cv, "mm");
	echo("front top main wing", front_top_main_wing, "mm");
	echo("back top main wing", back_top_main_wing, "mm");
	echo("front middle main wing", front_middle_main_wing, "mm");
	echo("back middle main wing", back_middle_main_wing, "mm");
	echo("front main wing support", front_main_wing_support, "mm");
	echo("back main wing support", back_main_wing_support, "mm");
}

//--------------------------------
// TRUE NACA 2412 AIRFOIL PROFILE
//--------------------------------
// One surface point of the NACA 4-digit section at chordwise station x (0..1).
// s = +1 for the upper surface, -1 for the lower surface.
function naca4_pt(x, s, m = 0.02, p = 0.4, t = 0.12) =
    let (
        yt = 5 * t * (0.2969*sqrt(x) - 0.1260*x - 0.3516*x*x + 0.2843*x*x*x - 0.1015*x*x*x*x),
        yc = (x < p) ? (m/p/p*(2*p*x - x*x)) : (m/(1-p)/(1-p)*((1 - 2*p) + 2*p*x - x*x)),
        dyc_dx = (x < p) ? (2*m/p/p*(p - x)) : (2*m/(1-p)/(1-p)*(p - x)),
        th = atan(dyc_dx)
    )
    [x - s*yt*sin(th), yc + s*yt*cos(th)];

// Closed NACA 2412 outline, counter-clockwise: upper surface TE -> LE,
// then lower surface LE -> TE (LE point not duplicated). Shared by the
// ribs and the wing mold so both use exactly the same section.
function naca2412_pts(chord, n = 50) = [
    for (i = [n:-1:0]) chord * naca4_pt(i/n,  1),
    for (i = [1:n])    chord * naca4_pt(i/n, -1)
];

module airfoil_profile(chord) {
    // m = 2% max camber at p = 40% chord, t = 12% thickness (see naca4_pt)
    polygon(points = naca2412_pts(chord));
}

//--------------------------------
// AIRFOIL RIB MODULE
//--------------------------------
module airfoil_rib(span_pos = 0, thickness = wall, c_length) {
    // Solid airfoil rib (no diamond or lightening holes)
    translate([0,0,-thickness/0.8]) linear_extrude(thickness*2.5)
		difference(){
			airfoil_profile(c_length);
			offset(delta = -2)
				airfoil_profile(c_length);
			}
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
        h=[h_joint, h_joint, h_joint],
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
	// Add airfoil ribs into this joint
	translate([0, 0, 0]) 
		rotate([90,0, 0])
			airfoil_rib(0, wall, c_root);
	//translate([b / 4, 0, 0]) airfoil_rib(1);
}

module joint_wing_top_rear() {      
    multi_joint(
        h=[h_joint, h_joint, h_joint, h_joint],
		azim=[90 - alpha, -90 + alpha, 0, 0],
		elev=[theta, theta, -90, 0],
		d=[di_08mm, di_08mm, di_08mm, di_08mm],
		through=[false, false, false, false],
		wall=[wall, wall, wall*1.2, wall],
		rounded=[true, true, true, true],
		webbing=[
		[0, 2, di_08mm/2, di_08mm/2 + wall, h_joint],
		[0, 3, -wall/2, wall/2, h_joint],
		[3, 1, -wall/2, wall/2, h_joint],
		[3, 2, -wall/2, wall/2, h_joint],
		[2, 1, di_08mm/2, di_08mm/2 + wall, h_joint]
		]);
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
    multi_joint_solid(
        h=[h_joint, h_joint, h_joint, h_joint],
        azim=[90 - alpha, -90 + alpha, 0, 0],
        elev=[19, 19, 90, -aoi],
        d=[di_08mm, di_08mm, di_08mm, [1.5, 1.5]],
        through=[false, false, false, true],
        wall=[wall, wall, 1.2*wall, wall],
        rounded=[true, true, true, false],
        webbing=[
            [2, 0, di_08mm/2, di_08mm/2 + wall, h_joint],
            [0, 3, -wall/2, wall/2, h_joint],
            [3, 1, -wall/2, wall/2, h_joint],
            [3, 2, -wall/2, wall/2, h_joint],
            [1, 2, di_08mm/2, di_08mm/2 + wall, h_joint],
        ]
    );
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
	// Add airfoil ribs into this joint
	translate([0, 0, 0]) 
		rotate([-90,180, 0])
			airfoil_rib(0, wall, cbar);
	module front_right_stuff() {
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

	rotate([-theta+8, 0, 0]) translate([-c_root*(1 - lambda*0.5), -bp/100000, 0]) front_right_stuff();
	
}

module joint_wing_top_front_left() {
    mirror([0, 1, 0]) joint_wing_top_front_right();
	// Add airfoil ribs into this joint
	
}

module joint_wing_top_rear_right() {
    
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
	// Add airfoil ribs into this joint
	translate([0, 0, 0]) 
		rotate([-90,180, 0])
			airfoil_rib(0, wall, c_tip);
	module tip_stuff() {
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
	x = di_08mm / 2 + wall;
	translate([-c_tip, 0, x-1]) rotate([180, 0, 105]) tip_stuff();
}

module joint_wing_top_front_left_tip() {
    mirror([0, 1, 0]) joint_wing_top_front_right_tip();
}

module joint_wing_top_rear_right_tip() {
    
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

module joint_T() {
    multi_joint(
    h = [h_joint, h_joint, h_joint],
    azim = [0, 90, 180],
    elev = [0, 0, 0],
    d = [di_08mm, di_08mm, di_08mm],
    through = [true, false, false],
    wall = [wall, wall, wall],
    rounded = [true, true, true],
    webbing = [
            [1, 0, di_08mm/2, di_08mm/2 + wall, h_joint],
            [2, 1, di_08mm/2, di_08mm/2 + wall, h_joint]
    ]);
}

module servo_arm(
        Wb = di_08mm + 2*wall,
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
            // webbing
            translate([-0.25, 0, thickness]) rotate([90, 0, 90]) linear_extrude(0.5)
                polygon([[0,0],[h_joint,0],[1,0.85*h_joint]]);
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
    
    // rudder
    translate([-x3, 0, 0]) rotate([0, 0, 0]) joint_gear_elevator();
    translate([0, 0, 2*h_joint]) rotate([90, 0, 0]) joint_front_rudder();
    translate([0, 0, hv+2*h_joint]) rotate([-90, 0, 0]) joint_right_ang();
    translate([-cr_tip, 0, hv+2*h_joint]) rotate([-90, 0, 180]) joint_right_ang();
    translate([-cr_tip, 0, 2*h_joint + cr_tip*tan(max_elev_deflect)]) rotate([90, 0, 180]) joint_back_rudder();
    translate([0, 0, -1.3*h_joint]) rotate([90, 0, 90]) joint_T();
    
    translate([0,0,hv + 2*h_joint]) rotate([0,90,0]) cf_rod([hv + 3.3*h_joint, 0.8]);
    translate([-cr_tip,0,hv + 2*h_joint]) rotate([0,90,0]) cf_rod([hv - cr_tip*tan(max_elev_deflect), 0.8]);
    translate([-cr_tip,0,hv + 2*h_joint]) rotate([0,0,0]) cf_rod([cr_tip, 0.8]);
    translate([-cr_tip,0,2*h_joint + cr_tip*tan(max_elev_deflect)]) rotate([180,max_elev_deflect,0]) cf_rod([cr_tip / cos(max_elev_deflect), 0.8]);
    
    translate([0,0,3*h_joint]) rotate([180,180,0]) servo_arm();
    
    
    // elevator
    translate([-x2, bh / 2, 0]) rotate([180, 0, 0]) joint_right_ang();
    translate([-x2, -bh / 2, 0]) rotate([0, 0, 0]) joint_right_ang();
    translate([-x2 - ch, bh / 2, 0]) rotate([180, 180, 0]) joint_right_ang();
    translate([-x2 - ch, -bh / 2, 0]) rotate([0, 0, 270]) joint_right_ang();
    
    translate([-x2,-bh / 2,0]) rotate([0,0,90]) cf_rod([bh, 0.8]);
    translate([-x2 - ch,-bh / 2,0]) rotate([0,0,90]) cf_rod([bh, 0.8]);
    translate([-x2 - ch,-bh / 2,0]) rotate([0,0,0]) cf_rod([ch, 0.8]);
    translate([-x2 - ch, bh / 2,0]) rotate([0,0,0]) cf_rod([ch, 0.8]);
    
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
    
    // Tail
    [hv + 3.3*h_joint, 0.8], // 13
    [hv - cr_tip*tan(max_elev_deflect), 0.8], // 14
    [cr_tip, 0.8], //15
    [cr_tip / cos(max_elev_deflect) , 0.8], //16
    [bh, 0.8], //17
    [bh, 0.8], //18
    [ch, 0.8], //19
    [ch, 0.8], //20
   
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
    translate([-c_root, 0, 0]) rotate([0, -theta + theta_rear_fix, 90-alpha]) cf_rod(cf_rods[1]);//Left back wing
    translate([-c_root, 0, 0]) rotate([0, -theta + theta_rear_fix, -90+alpha]) cf_rod(cf_rods[2]);//Right back wing
    translate([0, 0, 0]) rotate([0, -theta, 90]) cf_rod(cf_rods[3]);//Left Front wing
    translate([0, 0, 0]) rotate([0, -theta, -90]) cf_rod(cf_rods[4]);//Right front wing
    translate([-c_root, 0, -fuse_z + c_root*tan(aoi)]) rotate([0, -90, 0]) cf_rod(cf_rods[5]);//Back middle wing
    translate([0, 0, -fuse_z]) rotate([0, -90, 0]) cf_rod(cf_rods[6]);//Front middle Wing

    translate([0,  0, -fuse_z]) rotate([0, lg_theta, 90])  steel_rod(steel_rods[0]);
    translate([0,  0, -fuse_z]) rotate([0, lg_theta, -90])  steel_rod(steel_rods[1]);

    rotate([0, aoi, 0]) translate([10, 0, -fuse_z]) cf_square_rod(cf_square_rods[0]);

    translate([0, 50, -fuse_z-50]) rotate([0, 0, 180]) wheel(d=40);
    translate([0, -50, -fuse_z-50]) wheel(d=40);
  
    translate([-lh, 0, -fuse_z]) wheel(d=30);

    translate([0, 0, -fuse_z]) rotate([0, -theta -down_angle, 90]) cf_rod(cf_rods[7]);//Left Front Support Wing
    translate([0, 0, -fuse_z]) rotate([0, -theta -down_angle, -90]) cf_rod(cf_rods[8]);//Right Front Support Wing

    translate([-c_root, 0, -fuse_z + c_root*tan(aoi)]) rotate([0, -theta -down_angle_rear, 90-alpha]) cf_rod(cf_rods[7]);//Left Back Support Wing
    translate([-c_root, 0, -fuse_z + c_root*tan(aoi)]) rotate([0, -theta -down_angle_rear, -90+alpha]) cf_rod(cf_rods[8]);//Right Back Support Wing
 
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
    
    
    rotate([theta, 0, 0]) translate([-c_tip, bp/2, 0]) joint_wing_top_rear_left_tip();
    rotate([-theta, 0, 0]) translate([-c_tip, -bp/2, 0]) joint_wing_top_rear_right_tip();

    translate([0, 0, 0]) rotate([0, 0, 180]) joint_wing_top_front();
    translate([-c_root, 0, 0]) joint_wing_top_rear();

    translate([-c_root, 0, -fuse_z + 2*c_root*tan(aoi)]) rotate([0, 0, 0]) eyelet();
    
    x1 = -wall - 1.5/2;
    x2 = x1 - di_08mm/2;
    x3 = x2 - 1.5*di_08mm - wall/2;
    
    translate([-lh + x3, 0, -fuse_z + lh*tan(aoi)]) rotate([0, aoi, 0]) Tail();
    
    
 
}
module connector() {
	difference() {
        union() {
            translate([1, 0, 0]) cube([h_joint*2.5, 3, 3], true);
            multi_joint_solid(
                h=[h_joint],
                azim=[0],
                elev=[0],
                d=[[1.6, 1.6]],
                through=[true],
                wall=[wall],
                rounded=[false]);
        }
        multi_joint_drill_holes(
            h=[h_joint],
            azim=[0],
            elev=[0],
            d=[[1.5, 1.5]],
            through=[true],
            wall=[wall],
            rounded=[false]);
        
        
    }
}
module eyelet(
    wall = wall,
    di = di_08mm +.2,
    rod_clearance = 0.25,      // friction-fit tolerance
    h = h_joint,               // height matches other joints
    offset_extra = 1.0         // additional distance prevents intrusion of horizontal rod holes into vertical hole
) {

    // Vertical rod hole is same diameter as di_08mm
    rod_d = di;

    // Cylinder sizes
    vertical_outer = rod_d + 2*wall;
    side_outer = rod_d + 2*wall;

    // Compute offset needed to avoid intrusion
    // Ensures horizontal cylinders do not cut into vertical hole.
    offset = (vertical_outer/6) + (side_outer/6) + offset_extra;

    difference() {
        union() {

            // -------------------------------
            // Main vertical friction-fit tube
            // -------------------------------
            cylinder(h = h, d = vertical_outer, center = true);

            // -------------------------------
            // FRONT horizontal cylinder
            // -------------------------------
            translate([0, offset, 0])
                rotate([0,90,0])    // horizontal (X-axis)
                    cylinder(h = side_outer*1.2, d = side_outer, center = true);

            // -------------------------------
            // BACK horizontal cylinder
            // -------------------------------
            translate([0, -offset, 0])
                rotate([0,90,0])    // horizontal (X-axis)
                    cylinder(h = side_outer*1.2, d = side_outer, center = true);
        }

        // -------------------------------
        // Drill vertical rod hole
        // -------------------------------
        cylinder(h = h+1, d = di, center = true);

        // -------------------------------
        // Drill horizontal holes (same diameter!)
        // -------------------------------
        translate([0, offset, 0])
            rotate([0,90,0])
                cylinder(h = side_outer*1.4, d = di, center = true);

        translate([0, -offset, 0])
            rotate([0,90,0])
                cylinder(h = side_outer*1.4, d = di, center = true);
    }
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
        union(){
        linear_extrude(thick) square([305, (n+1)*space]);
        translate([-20, 0, 0]) linear_extrude(thick - 1) square([20, (n+1)*space]); //lip for sawing
        }
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
        cf_rods_odered = [5, 15, 6, 16, 20, 19, 14, 11, 12, 13, 9, 10, 7, 8, 0, 17, 18, 3, 4, 1, 2, 19, 20];
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
// WING MOLD
//--------------------------------
// Two-part clamshell mold for the main wing: NACA 2412 section
// (naca2412_pts), straight leading edge, linear taper c_root -> c_tip over
// the semi-span bp/2, and dihedral theta per side -- the same geometry the
// rods, joints and ribs above use.
//
// Each half-wing is built in a local frame:
//   x = chordwise (LE at 0, TE at +chord)
//   y = along the semi-span, following the dihedral (root 0 -> tip bp/2)
//   z = normal to the chord plane
// The parting surface is the chord plane, so every mold half has a flat back
// and pulls straight off the part. The root face is cut on the aircraft
// symmetry plane (tilted by theta in this frame), so a right mold and a left
// (mirrored) mold dowel together into one cavity with the full dihedral.
// Each side is split spanwise into segments that fit on the print bed.

mold_side         = "both";  // "right", "left" or "both"
mold_seg          = -1;      // segment to output for printing, -1 = all (0 = root)
mold_max_seg      = 200;     // max spanwise length of one printed segment, mm
mold_margin       = 12;      // flange around the planform (LE, TE and tip), mm
mold_back         = 6;       // material behind the deepest point of the cavity, mm
mold_tip_ext      = 1;       // cavity clearance past the tip for the tip rib/joint, mm
mold_n_span       = 24;      // spanwise sections in the lofted cavity
mold_rod_channels = true;    // grooves to co-mold the LE and TE carbon rods
mold_bolt_d       = 3.4;     // clamp bolt clearance (M3)
mold_bolt_pitch   = 40;      // max spacing between clamp bolts, mm
mold_key_d        = 5;       // registration cone base diameter
mold_key_h        = 2.5;     // registration cone height
mold_key_clear    = 0.2;     // socket clearance for the cones
mold_dowel_d      = 3.2;     // segment-joining dowel hole (3 mm pins)
mold_dowel_depth  = 8;       // dowel hole depth into each face
mold_explode      = 25;      // gap between halves in the "wing_mold" view
mold_explode_seg  = 6;       // gap between segments in the "wing_mold" view

// derived
mold_L        = bp/2;  // semi-span along the dihedral line
mold_up       =  c_root * max([for (q = naca2412_pts(1)) q[1]]);  // cavity height above chord plane (root)
mold_dn       = -c_root * min([for (q = naca2412_pts(1)) q[1]]);  // cavity depth below chord plane (root)
mold_zt       = mold_up + mold_back;  // top half thickness
mold_zb       = mold_dn + mold_back;  // bottom half thickness
mold_root_ext = max(mold_zt, mold_zb) * tan(theta) + 1;  // overshoot past the slanted root cut
mold_y_end    = mold_L + mold_tip_ext + mold_margin;      // outer end of the block at the tip
mold_nseg     = ceil(mold_y_end / mold_max_seg);
mold_edge_keep = mold_dowel_depth + mold_bolt_d;  // keep bolts/keys clear of segment faces
mold_B        = 4 * (mold_y_end + c_root + 50);  // "big" size for half-space cutters

if (do_echo) {
    echo("wing mold: semi-span", mold_L, "mm, segments per side", mold_nseg);
    echo("wing mold: segment length", mold_y_end / mold_nseg, "mm, root width", c_root + 2*mold_margin, "mm");
    echo("wing mold: top half thickness", mold_zt, "mm, bottom half thickness", mold_zb, "mm");
}

// chord at local span position y (LE straight, linear taper)
function mold_chord(y) =
    y <= 0 ? c_root :
    y >= mold_L ? c_tip :
    c_root - (c_root - c_tip) * y / mold_L;

// spanwise segment boundaries; segment k spans [mold_seg_y(k), mold_seg_y(k+1)]
function mold_seg_y(k) =
    k <= 0 ? -mold_root_ext - 1 :
    k >= mold_nseg ? mold_y_end :
    mold_y_end * k / mold_nseg;

// bolt (mid = false) or registration key (mid = true) stations in segment k
function mold_row_ys(k, mid = false) =
    let (
        a = (k == 0 ? mold_root_ext : mold_seg_y(k)) + mold_edge_keep,
        b = mold_seg_y(k + 1) - mold_edge_keep,
        n = max(1, ceil((b - a) / mold_bolt_pitch))
    )
    mid ? [for (i = [0:n-1]) a + (b - a) * (i + 0.5) / n]
        : [for (i = [0:n])   a + (b - a) * i / n];

function mold_x_le() = -mold_margin / 2;
function mold_x_te(y) = mold_chord(y) + mold_margin / 2;

// lofted wing solid through the span stations ys (local frame)
module wing_loft(ys) {
    sec = naca2412_pts(1);
    np = len(sec);
    ns = len(ys) - 1;
    pts = [for (y = ys) for (q = sec) [mold_chord(y) * q[0], y, mold_chord(y) * q[1]]];
    faces = concat(
        [[for (j = [np-1:-1:0]) j]],          // root cap
        [[for (j = [0:np-1]) ns*np + j]],     // tip cap
        [for (i = [0:ns-1]) for (j = [0:np-1])
            let (a = i*np + j, b = i*np + (j+1)%np, c = (i+1)*np + (j+1)%np, d = (i+1)*np + j)
            each [[a, b, c], [a, c, d]]]
    );
    polyhedron(points = pts, faces = faces, convexity = 4);
}

// wing side of the aircraft symmetry plane, in the local frame: y >= z*tan(theta)
module mold_root_keep() {
    rotate([-theta, 0, 0]) translate([-mold_B/2, 0, -mold_B/2]) cube(mold_B);
}

// one finished half-wing (local frame), trimmed at the symmetry plane
module wing_half_core() {
    intersection() {
        wing_loft(concat([-mold_root_ext], [for (i = [0:mold_n_span]) mold_L * i / mold_n_span]));
        mold_root_keep();
    }
}

// LE and TE rod grooves (rods at the LE point and along the TE, in the chord plane)
module mold_rod_grooves() {
    d = di_08mm;
    y0 = -mold_root_ext;
    y1 = mold_L + mold_tip_ext;
    translate([0, y0, 0]) rotate([-90, 0, 0]) cylinder(d = d, h = y1 - y0, $fn = 12);
    hull() { translate([c_root, y0, 0]) sphere(d = d, $fn = 12); translate([c_root, 0, 0]) sphere(d = d, $fn = 12); }
    hull() { translate([c_root, 0, 0]) sphere(d = d, $fn = 12); translate([c_tip, mold_L, 0]) sphere(d = d, $fn = 12); }
    hull() { translate([c_tip, mold_L, 0]) sphere(d = d, $fn = 12); translate([c_tip, y1, 0]) sphere(d = d, $fn = 12); }
}

module mold_cavity() {
    wing_loft(concat(
        [-mold_root_ext],
        [for (i = [0:mold_n_span]) mold_L * i / mold_n_span],
        [mold_L + mold_tip_ext]));
    if (mold_rod_channels) mold_rod_grooves();
}

// outer block: planform plus margin, flat backs parallel to the chord plane
module mold_block_half() {
    m = mold_margin;
    translate([0, 0, -mold_zb]) linear_extrude(mold_zb + mold_zt)
        polygon([
            [-m, -mold_root_ext],
            [c_root + m, -mold_root_ext],
            [c_root + m, 0],
            [c_tip + m, mold_L],
            [c_tip + m, mold_y_end],
            [-m, mold_y_end]
        ]);
}

module mold_bolt_holes() {
    for (k = [0:mold_nseg-1])
        for (y = mold_row_ys(k))
            for (x = [mold_x_le(), mold_x_te(y)])
                translate([x, y, -mold_zb - 1])
                    cylinder(d = mold_bolt_d, h = mold_zb + mold_zt + 2, $fn = 20);
}

// registration cones on the parting face (cl > 0 gives the matching sockets)
module mold_keys(cl = 0) {
    for (k = [0:mold_nseg-1])
        for (y = mold_row_ys(k, true))
            for (x = [mold_x_le(), mold_x_te(y)])
                translate([x, y, -eps])
                    cylinder(h = mold_key_h + cl + eps,
                             d1 = mold_key_d + 2*cl,
                             d2 = 0.4*mold_key_d + 2*cl, $fn = 24);
}

// full-length top (top = true) or bottom mold half, local frame
module mold_half(top) {
    B = mold_B;
    difference() {
        intersection() {
            mold_block_half();
            mold_root_keep();
            if (top) translate([-B/2, -B/2, 0]) cube(B);
            else     translate([-B/2, -B/2, -B]) cube(B);
        }
        mold_cavity();
        mold_bolt_holes();
        if (top) mold_keys(mold_key_clear);
    }
    if (!top) mold_keys(0);
}

// spanwise segment k of a mold half, with dowel holes in its joining faces
module mold_piece(k, top) {
    ya = mold_seg_y(k);
    yb = mold_seg_y(k + 1);
    zd = top ? mold_zt/2 : -mold_zb/2;
    B = mold_B;
    difference() {
        intersection() {
            mold_half(top);
            translate([-B/2, ya, -B/2]) cube([B, yb - ya, B]);
        }
        // faces shared with the neighbouring segment
        for (y = [ya, yb]) if (y > 0 && y < mold_y_end - eps)
            for (x = [mold_x_le(), mold_x_te(y)])
                translate([x, y, zd]) rotate([-90, 0, 0])
                    cylinder(d = mold_dowel_d, h = 2*mold_dowel_depth, center = true, $fn = 20);
        // slanted root face shared with the other side's mold
        if (k == 0)
            for (x = [mold_x_le(), mold_x_te(0)])
                translate([x, zd*tan(theta), zd]) rotate([-theta, 0, 0]) rotate([-90, 0, 0])
                    cylinder(d = mold_dowel_d, h = 2*mold_dowel_depth, center = true, $fn = 20);
    }
}

// local half-wing frame -> mold frame with dihedral (right = +y, left = -y)
module mold_side_place(side) {
    if (side == "left") mirror([0, 1, 0]) rotate([theta, 0, 0]) children();
    else rotate([theta, 0, 0]) children();
}

function mold_sides() = mold_side == "both" ? ["right", "left"] : [mold_side];

// both finished wing halves with dihedral, mold frame
module wing_core() {
    for (s = mold_sides()) mold_side_place(s) wing_half_core();
}

// same wing in the airframe frame used by assembly() (LE at x = 0, TE toward -x)
module wing_core_airframe() {
    rotate([0, 0, 180]) for (s = ["right", "left"]) mold_side_place(s) wing_half_core();
}

// exploded view of the mold around the wing
module wing_mold_view() {
    for (s = mold_sides()) mold_side_place(s) {
        for (k = [0:mold_nseg-1]) translate([0, k*mold_explode_seg, 0]) {
            color("SteelBlue", 0.85) translate([0, 0,  mold_explode]) mold_piece(k, true);
            color("Peru", 0.85)      translate([0, 0, -mold_explode]) mold_piece(k, false);
        }
        color("LightGrey") wing_half_core();
    }
}

// one piece, flat back on the bed and cavity facing up, starting at y = 0
module mold_print_piece(side, k, top) {
    ya = k == 0 ? 0 : mold_seg_y(k);
    yb = mold_seg_y(k + 1);
    if (side == "left") translate([0, yb, 0]) mirror([0, 1, 0]) mold_print_orient(top) mold_piece(k, top);
    else translate([0, -ya, 0]) mold_print_orient(top) mold_piece(k, top);
}

// top half flipped onto its back (same x footprint), bottom half lifted to z = 0
module mold_print_orient(top) {
    if (top) translate([c_root, 0, mold_zt]) rotate([0, 180, 0]) children();
    else     translate([0, 0, mold_zb]) children();
}

module wing_mold_print(top) {
    sides = mold_sides();
    segs = mold_seg < 0 ? [for (k = [0:mold_nseg-1]) k] : [mold_seg];
    pitch = c_root + 2*mold_margin + 10;
    for (si = [0:len(sides)-1]) for (ki = [0:len(segs)-1])
        translate([(si*len(segs) + ki) * pitch, 0, 0])
            mold_print_piece(sides[si], segs[ki], top);
}

//--------------------------------
// Parts Print Layout
//--------------------------------
module parts_print() {
    
    x = di_08mm / 2 + wall;
    
    translate([15, 5, 1.5]) rotate([0, 0, 0]) joint_gear_elevator();
    translate([15, 20, x]) rotate([0, 0, 0]) joint_T();
    translate([15, 35, x]) rotate([0, 0, 0]) joint_right_ang();
    translate([15, 55, x]) rotate([0, 0, 0]) joint_back_rudder();
    translate([15, 70, x]) rotate([180, 0, 0]) joint_wing_top_rear_left();
    translate([35, 5, x]) rotate([0, 0, 0]) joint_right_ang();
    translate([35, 20, x]) rotate([0, 0, 0]) joint_right_ang(); 
    translate([35, 35, x]) rotate([0, 0, 0]) joint_right_ang();
    translate([35, 55, x]) rotate([0, 270, 0]) joint_wing_top_front();
    translate([35, 75, x]) rotate([180, 0, 0]) joint_wing_top_rear_right();
    translate([55, 5, x]) rotate([0, 0, 0]) joint_right_ang();
    translate([55, 20, x]) rotate([0, 0, 0]) joint_right_ang();
    translate([55, 35, x]) rotate([0, 0, 0]) joint_right_ang();
    translate([55, 55, x]) rotate([0, 270, 0]) joint_wing_top_rear();
    translate([55, 70, x]) rotate([180, 0, 0]) joint_wing_top_rear_left_tip();
    translate([75, 5, x]) rotate([180, 0, 0]) joint_wing_top_front_left();
    translate([75, 25, x]) rotate([180, 0, 0]) joint_wing_top_front_right();
    translate([75, 35, x]) rotate([180, 0, 0]) joint_wing_top_front_left_tip();
    translate([75, 55, x]) rotate([180, 0, 0]) joint_wing_top_front_right_tip();
    
    translate([95, 10, x+wall]) rotate([0, 270, 0]) joint_wing_bottom_front();
    translate([95, 20, x]) rotate([0, 270, 270]) joint_wing_bottom_rear();
    translate([95, 35, 0]) rotate([0, 0, 270]) servo_arm();
    translate([95, 50, 0]) rotate([0, 0, 270]) servo_arm();
    translate([95, 50, 0]) rotate([0, 0, 270]) servo_arm();
    translate([25, 110, 0.5]) rotate([270, 0, 0]) wheel(d = 30);
    translate([75, 110, 0.5]) rotate([270, 0, 0]) wheel(d = 40);
    translate([125, 110, 0.5]) rotate([270, 0, 0]) wheel(d = 40);     
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
} else if (part == "rod_template") {
    rod_template();
} else if (part == "tail") {
    rotate([0, 0, 0]) Tail();
} else if (part == "parts_print") {
    rotate([0, 0, 0]) parts_print();
} else if (part == "servo") {
    rotate([0, 0, 0]) servo_arm();
} else if (part == "assembly") {
    rotate([0, -aoi, 0]) assembly();
} else if (part == "airfoils") {
	joint_wing_top_front();
	translate([0,50,0]) rotate([180,180,0]) joint_wing_top_front_right();
	translate([0,-50,0]) rotate([180,180,0]) joint_wing_top_front_left();
	translate([0,100,0]) rotate([180,180,0]) joint_wing_top_front_right_tip();
	translate([0,-100,0]) rotate([180,180,0]) joint_wing_top_front_left_tip();
} else if(part == "front joint") {
	rotate([0, 270, 0]) joint_wing_bottom_front();	
} else if (part == "mid"){
	joint_wing_top_front();
} else if (part == "eyelet"){
      eyelet();
} else if (part == "connector") {
	connector();
} else if (part == "airfoil left"){
	translate([0,-50,0]) rotate([180,180,0]) joint_wing_top_front_left();
} else if (part == "wing_mold") {
	wing_mold_view();                 // exploded mold around the wing
} else if (part == "wing_mold_top") {
	wing_mold_print(true);            // printable top halves (mold_side, mold_seg)
} else if (part == "wing_mold_bottom") {
	wing_mold_print(false);           // printable bottom halves (mold_side, mold_seg)
} else if (part == "wing_core") {
	wing_core();                      // the molded wing itself
} else if (part == "assembly_wing") {
	rotate([0, -aoi, 0]) {            // airframe with the wing overlaid, to check fit
		assembly();
		color("LightSkyBlue", 0.35) wing_core_airframe();
	}
} else {
    assert(false, str("unkonwn part: ", part));
}