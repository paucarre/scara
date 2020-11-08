'use strict';

const ComputeCameraView = function (canvas) {
    let frustrum = 10;
    let ratio = canvas.clientWidth / canvas.clientHeight;
    camera.orthoLeft = -ratio * frustrum / 2;
    camera.orthoRight = ratio * frustrum / 2;
    camera.orthoTop = frustrum / 2;
    camera.orthoBottom = -frustrum / 2;
}

const createScene = function (engine, canvas) {
    let scene = new BABYLON.Scene(engine);
    let camera = new BABYLON.FreeCamera("Camera", new BABYLON.Vector3(0,0,-1), scene);
    camera.attachControl(canvas, true);
    camera.inputs.addMouseWheel();
    camera.mode = BABYLON.Camera.ORTHOGRAPHIC_CAMERA;
    camera.position = new BABYLON.Vector3(0, 0, -100);
    camera.setTarget(BABYLON.Vector3.Zero());
    camera.minZ = -10000;
    camera.maxZ = 10000;
    let light = new BABYLON.HemisphericLight("HemiLight", new BABYLON.Vector3(0, 0, -2), scene);
    return scene;
};

class Robot {

    constructor(scene, red, green, blue) {
        this.red = red;
        this.green = green;
        this.blue = blue;
        this.scene = scene;
        this.axis_list = [];
        this.keys = ['angle_1', 'angle_2', 'angle_3'];
        this.topology_keys = ['l1', 'l2', 'l3'];
    }

    rotate_mesh_around_point(mesh, x , y, angle) {
        mesh.translate(new BABYLON.Vector3(x , y, 0), 1, BABYLON.Space.LOCAL);
        mesh.rotate(BABYLON.Axis.Z, angle, BABYLON.Space.LOCAL);
        mesh.translate(new BABYLON.Vector3(x , y, 0), -1, BABYLON.Space.LOCAL);
    };

    create_meshes_from_topology(topology) {
        for(var idx = 0; idx < this.topology_keys.length; idx ++) {
            let axis = BABYLON.MeshBuilder.CreateBox("Axis_", {height: topology[this.topology_keys[idx]], width: 5, depth: 5}, this.scene);
            let material = new BABYLON.StandardMaterial(this.scene);
            material.alpha = 1;
            material.diffuseColor = new BABYLON.Color3(this.red, this.green, this.blue);
            axis.material = material;
            this.axis_list.push(axis);
        }
    };

    draw(state, topology) {
        console.log(state);
        let parameters = state.parameters;
        let cartesian_solution = state.cartesian;
        var accumulated_angle = - (Math.PI / 2);
        let origni_distance = topology[this.topology_keys[0]] * 2;
        if(this.axis_list.length == 0) {
            this.create_meshes_from_topology(topology)
        }
        for(var idx = 0; idx < this.keys.length; idx ++) {
            let last_position = new BABYLON.Vector3(-(origni_distance) + cartesian_solution[idx][0], cartesian_solution[idx][1], 0);
            let axis = this.axis_list[idx];
            axis.rotation = new BABYLON.Vector3(0, 0, 0);
            axis.position = new BABYLON.Vector3(0, 0, 0);
            axis.translate(last_position, 1, BABYLON.Space.LOCAL);
            this.rotate_mesh_around_point(axis, 0, -topology[this.topology_keys[idx]] / 2, accumulated_angle + parameters[this.keys[idx]] );
            accumulated_angle = accumulated_angle + parameters[this.keys[idx]]
        }
    };

}


class RobotTopology {
    constructor(l1, l2, l3, h1) {
        this.l1 = l1;
        this.l2 = l2;
        this.l3 = l3;
        this.h1 = h1;
    }
}


const setup_form = function(id_form_selector, endpoint, success) {
    $(id_form_selector).submit(function(event){
        event.preventDefault();
        $.ajax({
            method: 'POST',
            url: endpoint,
            data : $(id_form_selector).serialize(),
            success: success,
            error: function(xhr, desc, err){
                console.log(err);
            }
        });
    });
};

const to_degrees = function(rads) {
    return (rads * 180) / Math.PI;
}

const setup_controller_connection = function(robot_state, robot_target) {
    const socket = io.connect( {transports: ['websocket']});
    socket.on('state_updated', function(data) {
        for (var state_type in data) {
            if (data.hasOwnProperty(state_type)) {
                const state = data[state_type];
                for (var axis in state.parameters) {
                    if (state.parameters.hasOwnProperty(axis)) {
                        const axis_div = document.getElementById(`${state_type}_${axis}`);
                        if(axis_div != null) {
                            //TODO: maybe make this more elegant (it's trying to check if it's an angle parameter)
                            let parameter_value = state.parameters[axis]
                            if(axis.includes('angle')) {
                                parameter_value = to_degrees(parameter_value);
                            }
                            axis_div.innerHTML = parameter_value.toFixed(2);
                        } else {
                            console.log(`*ERROR*: DIV element not found with ID '${state_type}_${axis}'`);
                        }
                    }
                }
            }
        }
        let tracker_state = null;
        if(data.tracker_state != null && data.tracker_state != undefined) {
            //TODO: this is approximate, better to get precise state when tracker updates
            tracker_state = data.tracker_state;
            robot_state.draw(data.current_state, data.topology);
            robot_target.draw(data.target_state, data.topology);
            //robot_drawer.last_tracker_state = data.tracker_state
            //robot_drawer.draw_states(data.topology, data.target_state, data.current_state,
             //   tracker_state, false);
        } else {
            robot_state.draw(data.current_state, data.topology);
            robot_target.draw(data.target_state, data.topology);
            //robot_drawer.draw_states(data.topology, data.target_state, data.current_state,
                //robot_drawer.last_tracker_state, true);
        }
    });
    socket.on('camera_updated', function(data) {
        const camera_image = document.getElementById("camera")
        camera_image.src= 'data:image/png;base64,' + data.camera_image;
    });
};


const setup_robot_view = function() {
    let parent = document.getElementById("canvas_wrapper");
    let canvas = document.getElementById("kinematics");
    canvas.width = parent.offsetWidth - 20;
    canvas.height = parent.offsetHeight - 20;
    let engine = new BABYLON.Engine(canvas, true);
    let scene = createScene(engine, canvas);
    ComputeCameraView(canvas);
    engine.runRenderLoop(function () {
        scene.render();
    });
    window.addEventListener("resize", function () {
        engine.resize();
    });
    let robot_state = new Robot(scene, 0, 0, 1);
    let robot_target = new Robot(scene, 0, 1, 0);
    return [robot_state, robot_target];
};

$(document).ready(function(){
    setup_form("#write_target_state_form", '/write_target_state', function(data) {});
    setup_form("#inverse_kinematics_form", '/inverse_kinematics', function(data) {
        $('#write_target_state_form input[name=angle_1]').val(data.angle_1);
        $('#write_target_state_form input[name=angle_2]').val(data.angle_2);
        $('#write_target_state_form input[name=angle_3]').val(data.angle_3);
        $('#write_target_state_form input[name=linear_1]').val(data.linear_1);
        $('#info_target_state_write').html(data.solution_type);
    });
    const [robot_state, robot_target] = setup_robot_view();
    setup_controller_connection(robot_state, robot_target);

});