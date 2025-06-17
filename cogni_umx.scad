$fn=16;

//tube_calibration(start=1.0, step=0.025, stop=2, thickness=1.5);

// find these using tube_calibration
di_07mm = 1.1;
di_1mm = 1.34;
tube_thickness = 1.3; // allows 2 ring walls
eps = 0.01; // small number below printing tolerance


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

// joint holding carbon fiber wing in rear
// one connection each side for 0.5mm carbon fiber, angled for dihedral
// one connection down for fuselage, 1mm

module rear_wing_top_joint() {
    multi_joint(
        h=[10, 10, 10],
        azim=[90, -90, 0],
        elev=[15, 15, -90],
        d=[di_07mm, di_07mm, di_1mm],
        through=[false, false, false],
        wall=[1.3, 1.3, 1.3],
        rounded=false);
}

rear_wing_joint();


module rear_wing_joint() {
    multi_joint(
        h=[10, 10, 10],
        azim=[90, -90, 0],
        elev=[15, 15, -90],
        d=[di_07mm, di_07mm, di_1mm],
        through=[false, false, false],
        wall=[1.3, 1.3, 1.3],
        rounded=false);
}
