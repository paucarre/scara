'use strict';

class RobotTopology {
    constructor(l1, l2, l3, h1) {
        this.l1 = l1;
        this.l2 = l2;
        this.l3 = l3;
        this.h1 = h1;
    }
}

class RobotDrawer {

    constructor(robot_topology, size) {
        this.robot_topology = robot_topology;
        this.size = size;
        this.max_distance = 1.2 * (this.robot_topology.l1 + this.robot_topology.l2 + this.robot_topology.l3);
        this.scale = this.size / (2.0 * this.max_distance);
        this.last_tracker_state = null;
    }

    scale_and_round(points) {
        return [( points[0] + this.max_distance ) * this.scale, ( points[1] - this.max_distance - 100) * this.scale];
    }

    to_rads(grads) {
        return (grads / 360) * (2 * Math.PI);
    }

    normalize(vector) {
        const norm = Math.sqrt( (vector[0] ** 2) + (vector[1] ** 2) );
        return [ vector[0] / norm, vector[1] / norm ];
    }

    distance(source, destination) {
        return [destination[0] - source[0], destination[1] - source[1]];
    }
    
    line(start, end, stroke_width, stroke_color, stroke_opacity, dashed) {
        let svg_dashed = '';
        if(dashed) {
            svg_dashed = `stroke-dasharray="4"`;
        }
        return `<line x1="${start[0]}" y1="${start[1]}" x2="${end[0]}" y2="${end[1]}" ${svg_dashed} style="stroke-opacity:${stroke_opacity};stroke:${stroke_color};stroke-width:${stroke_width}" />`
    }

    draw_angular(state, radius, color) {
        radius = radius / this.scale;
        let contents = '';
        let cartesian_solution = state.cartesian;
        for(let idx = 0; idx < cartesian_solution.length - 1; idx++) {

            // line
            let origin_point = this.scale_and_round(cartesian_solution[idx]);
            let destination_point = this.scale_and_round(cartesian_solution[idx + 1]);
            contents = contents + this.line([origin_point[0], -origin_point[1]], 
                [destination_point[0], -destination_point[1]],  2, color, 0.6, false);

            // center
            let source_direction = null;
            if(idx == 0) {
                source_direction = [0.0, 1.0];
            } else {
                source_direction = this.normalize( this.distance(cartesian_solution[idx - 1] , 
                    cartesian_solution[idx]) );
            }
            let destination_direction = this.normalize( this.distance(cartesian_solution[idx], 
                cartesian_solution[idx + 1]) );
            
            let source_point      = [cartesian_solution[idx][0], cartesian_solution[idx][1]];
            destination_point = [cartesian_solution[idx][0] + (radius * source_direction[0]), 
                cartesian_solution[idx][1] + (radius * source_direction[1]) ];
            source_point = this.scale_and_round(source_point);
            destination_point   = this.scale_and_round(destination_point);
            //contents = contents + this.line([source_point[0], -source_point[1]], 
            //    [destination_point[0], -destination_point[1]],  2, color, 0.4, true);

            // arc
            source_point      = [cartesian_solution[idx][0] + (radius * source_direction[0]), 
                cartesian_solution[idx][1] + (radius * source_direction[1])];
            destination_point = [cartesian_solution[idx][0] + (radius * destination_direction[0]), 
                cartesian_solution[idx][1] + (radius * destination_direction[1]) ];
            
            source_point   = this.scale_and_round(source_point);
            destination_point   = this.scale_and_round(destination_point);
            let angle = state.parameters[`angle_${idx + 1}`];
            let sign = angle > 0 ? 0 : 1;
            const path_description = `M ${source_point[0]},${-source_point[1]} a${radius * this.scale},${radius * this.scale} 0 0 ${sign} ${destination_point[0] - source_point[0]},${-destination_point[1] + source_point[1]}`
            //contents = contents + `<path d="${path_description}" stroke-dasharray="4" fill="none" style="stroke-opacity:${0.4};stroke:${color};stroke-width:${2}" />`
        }
        return contents;
    }

    draw_reachability() {
        let arm_len = this.robot_topology.l1 + this.robot_topology.l2 + this.robot_topology.l3;
        let radius = arm_len * this.scale;
        let source_point      = [0, 0];
        source_point   = this.scale_and_round(source_point);
        let contents = `<circle cx="${source_point[0]}" cy="${-source_point[1]}" r="${radius}" stroke="green" stroke-width="1" fill="None" />`;
        return contents;
    }

    draw_linear(linear_1, color) {
        const local_scale = (this.size / 2) / this.robot_topology.h1;
        let origin_point = [this.size + 1, (this.size / 2) - (linear_1 * local_scale)];
        let destination_point = [this.size + 30,  (this.size / 2) - (linear_1 * local_scale)];
        console.log('destination_point', destination_point)
        let contents = this.line(origin_point, destination_point, 2, color, 0.6, false);
        return contents;
    }

    draw_angular_square() {
        return `<rect fill="none" width="${this.size - 2}" height="${(this.size / 2) + 100}" style="stroke-width:1;stroke:rgb(0,0,0)" />`
    }
    
    draw_linear_square() {
        return `<rect fill="none" x="${this.size}" y="0" width="30" height="${(this.size / 2) + 100}" style="stroke-width:1;stroke:rgb(0,0,0)" />`
    }

    draw_state(state, radius, color) {
        let angular =  this.draw_angular(state, radius, color);
        let linear = '';//this.draw_linear(state.parameters['linear_1'], color);
        let angular_square = this.draw_angular_square();
        let linear_square = this.draw_linear_square();
        return angular + angular_square + linear + linear_square;
    }

    draw_tracker(tracker_state, dash_tracker) {
        const dash_config = dash_tracker ? 'stroke-dasharray="4"' : '';
        if(tracker_state != null) {

            // central point
            let radius = 5 * this.scale;
            let source_point = this.scale_and_round([tracker_state.x, tracker_state.y]);
            let position_circle = `<circle cx="${source_point[0]}" cy="${-source_point[1]}" r="${radius}" stroke="orange" stroke-width="1" fill="None" ${dash_config}/>`;
            
            let stroke_width = 1;
            
            // tangent line
            let start_tangent = [tracker_state.x + (100 * tracker_state.dy),
                tracker_state.y - (100 * tracker_state.dx)];
            let end_tangent = [tracker_state.x - (100 * tracker_state.dy),
                tracker_state.y + (100 * tracker_state.dx)];
            start_tangent   = this.scale_and_round(start_tangent);
            end_tangent   = this.scale_and_round(end_tangent);
            let orientation_line_tangent = `<line x1="${start_tangent[0]}" y1="${-start_tangent[1]}" 
                    x2="${end_tangent[0]}" y2="${-end_tangent[1]}" 
                    style="stroke:green;stroke-width:${stroke_width}" ${dash_config}/>`

            // orthogonal line
            let start_orthogonal = [tracker_state.x + (100 * tracker_state.dx),
                tracker_state.y + (100 * tracker_state.dy)];
            let end_orthogonal = [tracker_state.x - (100 * tracker_state.dx),
                tracker_state.y - (100 * tracker_state.dy)];
            start_orthogonal   = this.scale_and_round(start_orthogonal);
            end_orthogonal   = this.scale_and_round(end_orthogonal);
            let orientation_line_orthogonal = `<line x1="${start_orthogonal[0]}" y1="${-start_orthogonal[1]}" 
                    x2="${end_orthogonal[0]}" y2="${-end_orthogonal[1]}" 
                    style="stroke:orange;stroke-width:${stroke_width}" ${dash_config}/>`

            let linear = this.draw_linear(tracker_state.z, 'orange');
            const contents = `${linear}
                              ${position_circle}
                              ${orientation_line_tangent}
                              ${orientation_line_orthogonal}`;
            return contents;
        } else{
            return '';
        }
    }

    draw_states(target_state, current_state, tracker_state, dash_tracker) {
        const kinematics_image = document.getElementById("kinematics");
        const blue = 'rgb(0, 0, 255)';
        const svg_current_state = robot_drawer.draw_state(current_state, 50, blue);
        const red = 'rgb(255, 0, 0)';
        const svg_target_state = robot_drawer.draw_state(target_state, 50, red);
        const svg_reachability = robot_drawer.draw_reachability();
        let svg_tracker = ''
        if(tracker_state != null && tracker_state != undefined) {
            svg_tracker = robot_drawer.draw_tracker(tracker_state.parameters, dash_tracker);
        }
        kinematics_image.innerHTML = `
            <svg width="${robot_drawer.size + 100}px" height="${(robot_drawer.size / 2) + 120}px">
                ${svg_current_state}
                ${svg_target_state}
                ${svg_reachability}
                ${svg_tracker}
            </svg>`;
    };

}

const robot_drawer = new RobotDrawer(new RobotTopology(142.5, 142.5, 142.5 + 19.0, 290), 700);

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

const setup_controller_connection = function() {
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
            robot_drawer.last_tracker_state = data.tracker_state
            robot_drawer.draw_states(data.target_state, data.current_state, 
                tracker_state, false);
        } else {
            robot_drawer.draw_states(data.target_state, data.current_state, 
                robot_drawer.last_tracker_state, true);
        }
    });
    socket.on('camera_updated', function(data) {
        const camera_image = document.getElementById("camera")
        camera_image.src= 'data:image/png;base64,' + data.camera_image;
    });
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
    setup_controller_connection();
});