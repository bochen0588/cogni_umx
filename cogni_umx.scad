$fn=16;


module multi_joint(h, azim, elev, d, through, wall, rounded=false) {
    // Creates a joint to connection multiple carbon fiber rods
    
    // given lists of equal length (h, azim, elev, d, through, wall)
    // h: length of each joint
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
tube_thickness = 1.3; // allows 2 ring walls
eps = 0.01; // small number below printing tolerance

fuse_z = 50;  // fuselage height
theta = 6; // dihedral angle, deg (per wing, total is *2)
lambda = 0.5; // taper ratio
wing_loading = 3.45; // g/dm^2
m = 25; // flying mass, g,  with additional mocap payload
AR = 8; // aspect ratio
h_joint = 7; // joint length
wall = 1.3; // wall thickness
S = m/(wing_loading/1e4);  // wing area, g/dm^2
rounded_joints = false; // rounded joints
lg_theta = 45; // landing gear angle

b = sqrt(AR*S); // wing span, mm (projected)
bp = b/cos(theta); // actual length of wing

c_root = 2*S/(b*(1 + lambda)); // chord at root, mm
c_tip = lambda*c_root; // chord at tip, mm

echo("wing area", S*1e-4, "dm^2");
echo("wing span", b, "mm");
echo("chord root", c_root, "mm");
echo("chord tip", c_tip, "mm");

alpha = atan2(c_root - c_tip, b);

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

module wing_bottom_front_joint() {
    multi_joint(
        h=[h_joint, h_joint, h_joint, h_joint, h_joint, h_joint],
        azim=[90, -90, 0, 0, 90, -90],
        elev=[theta, theta, 90, 0, -lg_theta, -lg_theta],
        d=[di_07mm, di_07mm, di_1mm, [1.5, 1.5], di_1mm, di_1mm],
        through=[false, false, false, true, false, false],
        wall=[wall, wall, wall, wall, wall, wall, wall, wall],
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

module assembly() {

    translate([0, 0, 0]) rotate([0, 0, 180]) wing_top_front_joint();
    translate([-c_root, 0, 0]) rotate([0, 0, 0]) wing_top_rear_joint();
    rotate([0, 0, 180]) translate([0, 0, -fuse_z]) wing_bottom_front_joint();

    cf_rods = [
        // length, diameter (mm)
        [c_root, 1],
        [bp/(4*cos(alpha)), 1],
        [bp/(4*cos(alpha)), 1],
        [bp/4, 1],
        [bp/4, 1],
        [fuse_z, 1],
        [fuse_z, 1],
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
    translate([-c_root, 0, 0]) rotate([0, -theta, 90-alpha]) cf_rod(cf_rods[1]);
    translate([-c_root, 0, 0]) rotate([0, -theta, -90+alpha]) cf_rod(cf_rods[2]);
    translate([0, 0, 0]) rotate([0, -theta, 90]) cf_rod(cf_rods[3]);
    translate([0, 0, 0]) rotate([0, -theta, -90]) cf_rod(cf_rods[4]);
    translate([-c_root, 0, -fuse_z]) rotate([0, -90, 0]) cf_rod(cf_rods[5]);
    translate([0, 0, -fuse_z]) rotate([0, -90, 0]) cf_rod(cf_rods[6]);

    translate([0,  0, -fuse_z]) rotate([0, lg_theta, 90])  steel_rod(steel_rods[0]);
    translate([0,  0, -fuse_z]) rotate([0, lg_theta, -90])  steel_rod(steel_rods[1]);

    translate([30, 0, -fuse_z]) cf_square_rod(cf_square_rod[0]);

    translate([0, 50, -fuse_z-50]) rotate([0, 0, 180]) wheel(d=40);
    translate([0, -50, -fuse_z-50]) wheel(d=40);
}

translate([0, 0, 120]) rotate([0, 0, 0]) assembly();