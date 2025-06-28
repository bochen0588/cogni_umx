
//--------------------------------
// PARAMETERS
//--------------------------------
$fn=16;

part = "assembly"; // default to assembly
do_echo = false;

VER_MAJOR = 0;
VER_MINOR = 1;

di_08mm = 1.1;
eps = 0.01; // small number below printing tolerance

fuse_z = 50;  // fuselage height
theta = 6; // dihedral angle, deg (per wing, total is *2)
lambda = 0.5; // taper ratio
wing_loading = 3.45; // g/dm^2
m = 25; // flying mass, g,  with additional mocap payload
AR = 4.52; // aspect ratio, fune tuning to make longest arm be 300 mm
h_joint = 7; // joint length
wall = 0.5; // wall thickness
rounded_joints = false; // rounded joints
lg_theta = 45; // landing gear angle
deck_angle = 12; // tail dragger deck angle
tail_arm = 280;  // tail arm usually 2-3x mac
aoi = 5; // angle of incidence with thrust axis

// TODO: auto calculate these using geometry
down_angle = 18.5;
down_angle_rear = 12.4;
theta_rear_fix = 0.18;

//--------------------------------
// CALCULATED VALUES
//--------------------------------

S = m/(wing_loading/1e4);  // wing area, g/dm^2
b = sqrt(AR*S); // wing span, mm (projected)
bp = b/cos(theta); // actual length of wing

c_root = 2*S/(b*(1 + lambda)); // chord at root, mm
c_tip = lambda*c_root; // chord at tip, mm
mac = (c_root + c_tip)/2;
title = str("COGNI_UMX v", VER_MAJOR, ".", VER_MINOR);
alpha = atan2(c_root - c_tip, b/2);

if (do_echo) {
    echo(title);
    echo("wing area", S*1e-4, "dm^2");
    echo("wing span", b, "mm");
    echo("chord root", c_root, "mm");
    echo("chord tip", c_tip, "mm");
}

//--------------------------------
// MODULES
//--------------------------------

module multi_joint(h, azim, elev, d, through, wall, rounded=false, projection=false) {
    // Creates a joint to connection multiple carbon fiber rods
    
    // given lists of equal length (h, azim, elev, d, through, wall)
//    // h: length of each joint
    // azim: azimuth angles for each joint
    // elev: elevation angles of each joint
    // d: inner diameter for carbon fiber rods, if given list of 2, cuts a square
    // through: (whether or not hole goes all the way through)
    
    // wall: wall thickness
    // rounded: whether to use round or square joint, square better for printing
    
    module solid() {
        for (i = [0:len(h)-1]) {
            hi = h[i];
            r = eps;
            azimi = azim[i];
            elevi = elev[i];
            di = is_list(d[i]) ? max(d[i]) : d[i];
            walli = wall[i];
            ro = walli + di/2;
            minkowski() {
                rotate([0, 90 - elevi, azimi]) cylinder(h=hi, r=r);
                if (rounded) {
                    sphere(r=ro);
                } else {
                    cube(size=2*ro, center=true);
                }
            }
        }
    }
    
    module drill_holes() {
        for (i = [0:len(h)-1]) { // cylinderical hole for each joint
            hi = h[i];
            r = 0.001;
            azimi = azim[i];
            elevi = elev[i];
            di = is_list(d[i]) ? max(d[i]) : d[i];
            walli = wall[i];
            throughi = through[i];
            ro = walli + di/2;
            rotate([0, 90 - elevi, azimi]) union() {
                if (is_list(d[i])) {
                    if (through[i]) {
                        translate([0, 0, 0]) cube([d[i][0],  d[i][1], 2*hi], center=true);
                    } else {
                        translate([0, 0, hi/2]) cube([d[i][0],  d[i][1], hi], center=true);
                    }
                } else {
                    if (through[i]) {
                        translate([0, 0, -hi]) cylinder(h=2*hi, d=di); // cut tube for rod
                    } else {
                        cylinder(h=hi, d=di); // cut tube for rod
                    }
                }
                // cut off end
                translate([0, 0, hi + ro])
                cube([4*ro, 4*ro, eps+2*ro], center=true);
            }
        }
    }
    
    difference() {
        solid();
        drill_holes();
    }
}

module joint_wing_top_front() {
    delta =  di_08mm/2 + wall;
    s = 0.35; // todo base on angle
    multi_joint(
        h=[h_joint, h_joint, h_joint, h_joint],
        azim=[90, -90, 0, 0],
        elev=[theta, theta, -90, 0],
        d=[di_08mm, di_08mm, di_08mm, di_08mm],
        through=[false, false, false, false],
        wall=[wall, wall, wall, wall],
        rounded=rounded_joints);
    module fin() {
        translate([-di_08mm/2 - wall, 0,  delta])
            rotate([0, 90, 0]) linear_extrude(wall) polygon([
                [0, 0], [h_joint + delta, 0], [wall, h_joint - wall], [0, h_joint - wall], [0, h_joint]]);
    }
    fin();
    mirror([0, 1, 0]) fin();
}

module joint_wing_top_rear() {
    delta =  di_08mm/2 + wall;
    s = 0.35; // todo base on angle
    multi_joint(
        h=[h_joint, h_joint, h_joint, h_joint],
        azim=[90 - alpha, -90 + alpha, 0, 0],
        elev=[theta, theta, -90, 0],
        d=[di_08mm, di_08mm, di_08mm, di_08mm],
        through=[false, false, false, false],
        wall=[wall, wall, wall, wall],
        rounded=rounded_joints);
    module fin() {
        translate([-di_08mm/2 - wall, 0,  delta])
            rotate([0, 90, 0]) linear_extrude(wall) polygon([
                [0, 0], [h_joint + delta, 0], [wall, h_joint], [0, h_joint], [0, h_joint]]);
        translate([-di_08mm/2 - wall, 0,  wall])
            rotate([0, 0, 0]) linear_extrude(wall) polygon([
                [0, 0],  [wall, 0], [h_joint*s, h_joint-wall], [0, h_joint],]);
    }
    fin();
    mirror([0, 1, 0]) fin();
}

module joint_wing_bottom_front() {
    module joint() {
        multi_joint(
            h=[h_joint, h_joint, h_joint, h_joint, h_joint, h_joint],
            azim=[90, -90, 0, 0, 90, -90],
            elev=[25, 25, 90, aoi, -lg_theta, -lg_theta],
            d=[di_08mm, di_08mm, di_08mm, [1.5, 1.5], di_08mm, di_08mm],
            through=[false, false, false, true, false, false],
            wall=[wall, wall, wall, wall, wall, wall, wall, wall],
            rounded=rounded_joints);
    }
    
    // cut joint flat
    difference() {
        joint();
        translate([-(5 + di_08mm/2 + wall), 0, 0]) cube([10, 100, 100], center=true);
    }
}

module joint_wing_bottom_rear() {
    delta =  (1.5 + wall)/2;
    s1 = 0.7; // TODO
    s2 = 0.35;
    multi_joint(
        h=[h_joint, h_joint, h_joint, h_joint],
        azim=[90 - alpha, -90 + alpha, 0, 0],
        elev=[19, 19, 90, -aoi],
        d=[di_08mm, di_08mm, di_08mm, [1.5, 1.5]],
        through=[false, false, false, true],
        wall=[wall, wall, wall, wall, wall, wall],
        rounded=rounded_joints);
    module fin() {
        translate([-di_08mm/2-wall/2, 0,  delta])
            rotate([0, -90, 0]) linear_extrude(wall) polygon([
                [0, 0], [h_joint*s1 + delta, 0], [h_joint*s1 + delta, wall], [wall, h_joint], [0, h_joint], [0, h_joint]]);
        translate([-di_08mm/2 - wall, 0,  delta])
            rotate([0, 0, 0]) linear_extrude(wall) polygon([
                [0, 0],  [wall, 0], [h_joint*s2, h_joint-wall], [0, h_joint],]);
    }
    fin();
    mirror([0, 1, 0]) fin();
}

module joint_wing_top_front_right() {
    multi_joint(
        h=[h_joint, h_joint, h_joint],
        azim=[90, 90, 180],
        elev=[-down_angle, 0, 0],
        d=[di_08mm, di_08mm, di_08mm],
        through=[false, true, false],
        wall=[wall, wall, wall],
        rounded=rounded_joints);
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
        rounded=rounded_joints);
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
        rounded=rounded_joints);
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
        rounded=rounded_joints);
}

module joint_wing_top_rear_left_tip() {
    mirror([0, 1, 0]) joint_wing_top_rear_right_tip();
}

module joint_gear_elevator() {
    multi_joint(
        h=[h_joint],
        azim=[0],
        elev=[0],
        d=[[1.5, 1.5]],
        through=[false],
        wall=[wall],
        rounded=rounded_joints);
    
    translate([0, 0.25, 1.5/2 + eps]) rotate([90, 0, 0]) linear_extrude(0.5)
    polygon([[x3 + di_08mm/2,0],[h_joint,0],[x3 + di_08mm/2,h_joint*0.7]]);

    x1 = -wall - 1.5/2;
    x2 = x1 - di_08mm/2;
    x3 = x2 - wall - di_08mm;
    delta = (1.5 - di_08mm)/2;
    translate([x2, 0, 0]) multi_joint(
        h=[h_joint/2, h_joint/2],
        azim=[90, -90],
        elev=[0, 0],
        d=[di_08mm, di_08mm],
        through=[true],
        wall=[wall+delta, wall+delta],
        rounded=rounded_joints);
    translate([x3, 0, -delta]) multi_joint(
        h=[h_joint],
        azim=[0],
        elev=[90],
        d=[di_08mm],
        through=[true],
        wall=[wall, wall],
        rounded=rounded_joints);
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

    rotate([0, aoi, 0]) translate([20, 0, -fuse_z]) cf_square_rod(cf_square_rods[0]);

    translate([0, 50, -fuse_z-50]) rotate([0, 0, 180]) wheel(d=40);
    translate([0, -50, -fuse_z-50]) wheel(d=40);
  
    translate([-tail_arm, 0, -fuse_z]) wheel(d=30);

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
    translate([-280, 0, -fuse_z + 280*tan(aoi)]) rotate([0, aoi, 0]) joint_gear_elevator();
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
    assembly();
} else {
    assert(false, str("unkonwn part: ", part));
}
