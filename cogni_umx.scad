$fn=16;

module multi_joint(h, azim, elev, d, through, wall, rounded=false) {
    // Creates a joint to connection multiple carbon fiber rods
    
    // given lists of equal length (h, azim, elev, d, through, wall)
//    // h: length of each joint
    // azim: azimuth angles for each joint
    // elev: elevation angles of each joint
    // d: inner diameter for carbon fiber rods, if given list of 2, cuts a square
    // through: (whether or not hole goes all the way through)
    
    // wall: wall thickness
    // rounded: whether to use round or square joint, square better for printing
    
    difference() {
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
        union() {
            sphere(d=max(d)); // round interior hole
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
    }
}

// find these using tube_calibration
di_07mm = 1.1;
di_1mm = 1.34;
eps = 0.01; // small number below printing tolerance

fuse_z = 50;  // fuselage height
theta = 6; // dihedral angle, deg (per wing, total is *2)
lambda = 0.5; // taper ratio
wing_loading = 3.45; // g/dm^2
m = 25; // flying mass, g,  with additional mocap payload
AR = 5; // aspect ratio
h_joint = 7; // joint length
wall = 1; // wall thickness
S = m/(wing_loading/1e4);  // wing area, g/dm^2
rounded_joints = true; // rounded joints
lg_theta = 45; // landing gear angle
deck_angle = 12; // tail dragger deck angle

down_angle = 17;

b = sqrt(AR*S); // wing span, mm (projected)
bp = b/cos(theta); // actual length of wing

c_root = 2*S/(b*(1 + lambda)); // chord at root, mm
c_tip = lambda*c_root; // chord at tip, mm
mac = (c_root + c_tip)/2;

tail_arm = 280;  // tail arm usually 2-3x mac

echo("wing area", S*1e-4, "dm^2");
echo("wing span", b, "mm");
echo("chord root", c_root, "mm");
echo("chord tip", c_tip, "mm");

alpha = atan2(c_root - c_tip, b/2);

module wing_top_front_joint() {
    multi_joint(
        h=[h_joint, h_joint, h_joint, h_joint],
        azim=[90, -90, 0, 0],
        elev=[theta, theta, -90, 0],
        d=[di_07mm, di_07mm, di_1mm, di_1mm],
        through=[false, false, false, false],
        wall=[wall, wall, wall, wall],
        rounded=rounded_joints);
}

module wing_top_rear_joint() {
    multi_joint(
        h=[h_joint, h_joint, h_joint, h_joint],
        azim=[90 - alpha, -90 + alpha, 0, 0],
        elev=[theta, theta, -90, 0],
        d=[di_07mm, di_07mm, di_1mm, di_1mm],
        through=[false, false, false, false],
        wall=[wall, wall, wall, wall],
        rounded=rounded_joints);
}

module wing_bottom_front_joint() {
    multi_joint(
        h=[h_joint, h_joint, h_joint, h_joint, h_joint, h_joint],
        azim=[90, -90, 0, 0, 90, -90],
        elev=[25, 25, 90, 0, -lg_theta, -lg_theta],
        d=[di_07mm, di_07mm, di_1mm, [1.5, 1.5], di_1mm, di_1mm],
        through=[false, false, false, true, false, false],
        wall=[wall, wall, wall, wall, wall, wall, wall, wall],
        rounded=rounded_joints);
}

module wing_bottom_rear_joint() {
    multi_joint(
        h=[h_joint, h_joint, h_joint, h_joint],
        azim=[90 - alpha, -90 + alpha, 0, 0],
        elev=[25, 25, 90, 0],
        d=[di_07mm, di_07mm, di_1mm, [1.5, 1.5]],
        through=[false, false, false, true],
        wall=[wall, wall, wall, wall, wall, wall],
        rounded=rounded_joints);
}

module wing_top_front_right() {
    multi_joint(
        h=[h_joint, h_joint, h_joint],
        azim=[90, 90, 180],
        elev=[-down_angle, 0, 0],
        d=[di_1mm, di_1mm, di_1mm],
        through=[true, false, false],
        wall=[wall, wall, wall],
        rounded=rounded_joints);
}

module wing_top_front_left() {
    mirror([0, 1, 0]) wing_top_front_right();
}

module wing_top_rear_right() {
    multi_joint(
        h=[h_joint, h_joint, h_joint],
        azim=[90+alpha, 90+alpha, 0],
        elev=[-18, 0, 0],
        d=[di_1mm, di_1mm, di_1mm],
        through=[true, false, false],
        wall=[wall, wall, wall],
        rounded=rounded_joints);
}

module wing_top_rear_left() {
    mirror([0, 1, 0]) wing_top_rear_right();
}

module wing_top_front_right_tip() {
    multi_joint(
        h=[h_joint, h_joint],
        azim=[90, 180],
        elev=[0, 0],
        d=[di_1mm, di_1mm],
        through=[false, false],
        wall=[wall, wall],
        rounded=rounded_joints);
}

module wing_top_front_left_tip() {
    mirror([0, 1, 0]) wing_top_front_right_tip();
}

module wing_top_rear_right_tip() {
    multi_joint(
        h=[h_joint, h_joint],
        azim=[0, 90 + alpha],
        elev=[0, 0],
        d=[di_1mm, di_1mm],
        through=[false, false],
        wall=[wall, wall],
        rounded=rounded_joints);
}

module wing_top_rear_left_tip() {
    mirror([0, 1, 0]) wing_top_rear_right_tip();
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

module wheel(d, spokes=8, thick=1, d_axle_cut=di_1mm) {
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

module rib_center() {
    //module right() {
    //    airfoil_elliptical(chord=c_root, span=b, lambda=lambda, width=5, thickness=0.4);
    //}
    //right();
    //translate([0, -eps, 0]) mirror([0, 1, 0]) right();
    translate([0, 0, 0]) rotate([0, 0, 180]) wing_top_front_joint();
    translate([-c_root, 0, 0]) rotate([0, 0, 0]) wing_top_rear_joint();
}

module rib_left_half() {
    translate([0, -b/4, b/4*tan(theta)]) rotate([theta, 0, 0]) airfoil_elliptical(chord=c_root, span=b, lambda=lambda, width=5, thickness=1);
}

module rib_right_half() {
    translate([0, b/4, b/4*tan(theta)]) rotate([theta, 0, 0]) airfoil_elliptical(chord=c_root, span=b, lambda=lambda, width=5, thickness=1);
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

module assembly() {
    rotate([0, 0, 180]) translate([0, 0, -fuse_z]) wing_bottom_front_joint();
    translate([-c_root, 0, -fuse_z]) wing_bottom_rear_joint();
    
    rotate([theta, 0, 0]) translate([0, b/4, 0]) wing_top_front_left();
    rotate([-theta, 0, 0]) translate([0, -b/4, 0]) wing_top_front_right();

    rotate([theta, 0, 0]) translate([0, b/2, 0]) wing_top_front_left_tip();
    rotate([-theta, 0, 0]) translate([0, -b/2, 0]) wing_top_front_right_tip();
    
    rotate([theta, 0, 0]) translate([-c_root*(1 - lambda*0.5), b/4, 0]) wing_top_rear_left();
    rotate([-theta, 0, 0]) translate([-c_root*(1 - lambda*0.5), -b/4, 0]) wing_top_rear_right();
    
    rotate([theta, 0, 0]) translate([-c_tip, b/2, 0]) wing_top_rear_left_tip();
    rotate([-theta, 0, 0]) translate([-c_tip, -b/2, 0]) wing_top_rear_right_tip();

    cf_rods = [
        // length, diameter (mm)
        [c_root, 1],
        [bp/(2*cos(alpha)), 1],
        [bp/(2*cos(alpha)), 1],
        [bp/2, 1],
        [bp/2, 1],
        [fuse_z, 1],
        [fuse_z, 1],
        [170, 1],
        [170, 1],
        [c_root/2, 1],
        [c_root/2, 1],
        [c_root*(1 - lambda*0.5), 1],
        [c_root*(1 - lambda*0.5), 1],
        [c_tip, 1],
        [c_tip, 1],
    ];

    steel_rods = [
        // length, diameter (mm)
        [70, 1],
        [70, 1],
    ];

    cf_square_rod = [
        // length, [width, height]
        [300, [1.5, 1.5]],
    ];

    translate([-c_root, 0, 0]) cf_rod(cf_rods[0]);
    translate([-c_root, 0, 0]) rotate([0, -theta + 0.18, 90-alpha]) cf_rod(cf_rods[1]);
    translate([-c_root, 0, 0]) rotate([0, -theta + 0.18, -90+alpha]) cf_rod(cf_rods[2]);
    translate([0, 0, 0]) rotate([0, -theta, 90]) cf_rod(cf_rods[3]);
    translate([0, 0, 0]) rotate([0, -theta, -90]) cf_rod(cf_rods[4]);
    translate([-c_root, 0, -fuse_z]) rotate([0, -90, 0]) cf_rod(cf_rods[5]);
    translate([0, 0, -fuse_z]) rotate([0, -90, 0]) cf_rod(cf_rods[6]);

    translate([0,  0, -fuse_z]) rotate([0, lg_theta, 90])  steel_rod(steel_rods[0]);
    translate([0,  0, -fuse_z]) rotate([0, lg_theta, -90])  steel_rod(steel_rods[1]);

    translate([20, 0, -fuse_z]) cf_square_rod(cf_square_rod[0]);

    translate([0, 50, -fuse_z-50]) rotate([0, 0, 180]) wheel(d=40);
    translate([0, -50, -fuse_z-50]) wheel(d=40);
    
    translate([-tail_arm, 0, -fuse_z]) wheel(d=30);

    translate([0, 0, -fuse_z]) rotate([0, -theta -down_angle, 90]) cf_rod(cf_rods[7]);
    translate([0, 0, -fuse_z]) rotate([0, -theta -down_angle, -90]) cf_rod(cf_rods[8]);

    translate([-c_root, 0, -fuse_z]) rotate([0, -theta -down_angle, 90-alpha]) cf_rod(cf_rods[7]);
    translate([-c_root, 0, -fuse_z]) rotate([0, -theta -down_angle, -90+alpha]) cf_rod(cf_rods[8]);
    
    translate([-c_root, 0, -fuse_z]) cf_rod(cf_rods[9]);
    translate([-c_root, 0, -fuse_z]) cf_rod(cf_rods[10]);
 
    rotate([theta, 0, 180]) translate([0, b/4, 0]) cf_rod(cf_rods[11]);
    rotate([-theta, 0, 180]) translate([0, -b/4, 0]) cf_rod(cf_rods[12]);
    
    rotate([theta, 0, 180]) translate([0, b/2, 0]) cf_rod(cf_rods[13]);
    rotate([-theta, 0, 180]) translate([0, -b/2, 0]) cf_rod(cf_rods[14]);
    
    rib_center();
    //rib_left_half();
    //rib_right_half();

    //airfoil_elliptical(chord=c_root, camber=0.08, resolution=30);
}


//echo(bp/2wd);

module printing_1() {
    wing_top_front_joint();
    //translate([10, 10, 0]) rotate([0, 90, 0]) wing_bottom_front_joint();
    //translate([-10, -10, 0]) rotate([0, 90, 0]) wing_top_rear_joint();
}

//rib_center();

//printing_1();

translate([0, 0, 120]) rotate([0, 0, 0]) assembly();