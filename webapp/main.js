var app = new Vue({
    el: '#app',
    // storing the state of the page
    
    data: {
        connected: false,
        ros: null,
        logs: [],
        loading: false,
        // 3D
        viewer: null,
        tfClient: null,
        urdfClient: null,
        // CAMERA
        cameraAddress: '',
        
        // NAV
        goal: null,
        action: {
            goal: {position: {x: 0, y: 0, z: 0}},
            feedback: {position: 0, state: 'idle'},
            result: {success: false},
            status: {status: 0, text: ''},
        },
        
    },
    // helper methods to connect to ROS
    methods: {
        connect: function() {
            this.loading = true
            this.ros = new ROSLIB.Ros({
                url: 'ws://localhost:9090'
            })
            this.ros.on('connection', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Connected!')
                this.connected = true
                this.loading = false
                this.pubInterval = setInterval(this.publish, 100)
                // MAP
                this.setupMap()
                // 3D
                this.setup3dViewer()
                // CAMERA
                this.setupCamera()
            })
            this.ros.on('error', (error) => {
                this.logs.unshift((new Date()).toTimeString() + ` - Error: ${error}`)
            })
            this.ros.on('close', () => {
                this.logs.unshift((new Date()).toTimeString() + ' - Disconnected!')
                this.connected = false
                this.loading = false
                clearInterval(this.pubInterval)
                // 3D
                this.unset3DViewer()
                // CAMERA
                this.unsetCamera()
            })
        },
        disconnect: function() {
            this.ros.close()
        },
        // MAP
        setupMap(){
            this.mapViewer = new ROS2D.Viewer({
                    divID: 'map',
                    width: 420,
                    height: 360,
                })
            this.mapGridClient = new ROS2D.OccupancyGridClient({
                ros: this.ros,
                rootObject: this.mapViewer.scene,
                continuous: true,
            })
            this.mapGridClient.on('change', () => {
                this.mapViewer.scaleToDimensions(this.mapGridClient.currentGrid.width, this.mapGridClient.currentGrid.height);
                this.mapViewer.shift(this.mapGridClient.currentGrid.pose.position.x, this.mapGridClient.currentGrid.pose.position.y)
            })   
        },
        unsetMap(){
            document.getElementById('map').innerHTML = ''
        },
        // 3D
        setup3dViewer() {
            this.viewer = new ROS3D.Viewer({
                background: '#cccccc',
                divID: '3DViewer',
                width: 400,
                height: 300,
                antialias: true,
                fixedFrame: 'odom'
            })
            this.viewer.addObject(new ROS3D.Grid({
                color: '#0181c4',
                cellSize: 0.5,
                num_cells: 20
            }))
            this.tfClient = new ROSLIB.TFClient({
                ros: this.ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0
            })
            this.urdfClient = new ROS3D.UrdfClient({
                ros: this.ros,
                param: 'robot_description',
                tfClient: this.tfClient,
                path: location.origin + location.pathname,
                rootObject: this.viewer.scene,
                loader: ROS3D.COLLADA_LOADER_2
            })           
        },
        unset3DViewer() {
            document.getElementById('3DViewer').innerHTML = ''
        },
        // CAMERA
        setupCamera: function() {
            let without_wss = this.rosbridge_address.split('wss://')[1]
            console.log(without_wss)
            let domain = without_wss.split('/')[0] + '/' + without_wss.split('/')[1]
            console.log(domain)
            let host = domain + '/cameras'
            let viewer = new MJPEGCANVAS.Viewer({
                divID: 'camera',
                host: host,
                width: 320,
                height: 240,
                topic: '/camera/image_raw',
                ssl: true,
            })
            
        },
        unsetCamera(){
            document.getElementById('camera').innerHTML = ''        
        },
        // JOYSTICK
        publish: function() {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: this.joystick.vertical, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: this.joystick.horizontal, },
            })
            topic.publish(message)
        },
        startDrag() {
            this.dragging = true
            this.x = this.y = 0
        },
        stopDrag() {
            this.dragging = false
            this.x = this.y = 'no'
            this.dragCircleStyle.display = 'none'
            this.resetJoystickVals()
        },
        doDrag(event) {
            if (this.dragging) {
                this.x = event.offsetX
                this.y = event.offsetY
                let ref = document.getElementById('dragstartzone')
                this.dragCircleStyle.display = 'inline-block'

                let minTop = ref.offsetTop - parseInt(this.dragCircleStyle.height) / 2
                let maxTop = minTop + 200
                let top = this.y + minTop
                this.dragCircleStyle.top = `${top}px`

                let minLeft = ref.offsetLeft - parseInt(this.dragCircleStyle.width) / 2
                let maxLeft = minLeft + 200
                let left = this.x + minLeft
                this.dragCircleStyle.left = `${left}px`

                this.setJoystickVals()
            }
        },
        setJoystickVals() {
            this.joystick.vertical = -1 * ((this.y / 200) - 0.5)
            this.joystick.horizontal = +1 * ((this.x / 200) - 0.5)
        },
        resetJoystickVals() {
            this.joystick.vertical = 0
            this.joystick.horizontal = 0
        },
        // NAV
        sendWaypoint: function(){

            switch (this.waypoint.index){
                case 1:
                    this.action.goal.position = this.waypoint.wp1

                    break;
                case 2:
                    this.action.goal.position = this.waypoint.wp2

                    break
                case 3:
                    this.action.goal.position = this.waypoint.wp3
                    break;
                case 4:
                    this.action.goal.position = this.waypoint.wp4
                    break;
                case 5:
                    this.action.goal.position = this.waypoint.wp5
                    break;
                case 6:
                    this.action.goal.position = this.waypoint.wp6
                    break;
                case 7:
                    this.action.goal.position = this.waypoint.wp7
                    break;
                case 8:
                    this.action.goal.position = this.waypoint.wp8
                    break;
                case 9:
                    this.action.goal.position = this.waypoint.wp9
                    break;
                case 10:
                    this.action.goal.position = this.waypoint.wp10
                    break;
            }


            this.sendGoal()
        },
        sendGoal: function() {
            let actionClient = new ROSLIB.ActionClient({
                ros : this.ros,
                serverName : '/tortoisebot_as',
                actionName : 'course_web_dev_ros/WaypointActionAction'
            })

            this.goal = new ROSLIB.Goal({
                actionClient : actionClient,
                goalMessage: {
                    ...this.action.goal
                }
            })

            this.goal.on('status', (status) => {
                this.action.status = status
            })

            this.goal.on('feedback', (feedback) => {
                this.action.feedback = feedback
            })

            this.goal.on('result', (result) => {
                this.action.result = result
            })

            this.goal.send()
        },
        cancelGoal: function() {
            this.goal.cancel()
        },
    },
    mounted() {
        // MAP
        this.interval = setInterval(() => {
            if (this.ros != null && this.ros.isConnected) {
                this.ros.getNodes((data) => { }, (error) => { })
            }
        }, 10000)
        // JOYSTICK
        window.addEventListener('mouseup', this.stopDrag)
    },
})