import math, struct, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from rclpy.time import Time

class TerrainMapper(Node):
    def __init__(self):
        super().__init__('terrain_mapper')
        self.declare_parameter('input_topic','/livox/lidar')
        self.declare_parameter('output_frame','odom')
        self.declare_parameter('resolution',0.10)
        self.declare_parameter('width',15.0)
        self.declare_parameter('height',15.0)
        self.declare_parameter('publish_rate',5.0)
        self.declare_parameter('min_points_per_cell',3)
        self.declare_parameter('ground_percentile',15.0)
        self.declare_parameter('max_height_step',0.18)
        self.declare_parameter('lethal_height_step',0.30)
        self.declare_parameter('max_sensor_age',0.35)
        self.frame=self.get_parameter('output_frame').value
        self.res=float(self.get_parameter('resolution').value)
        self.nx=max(1,int(self.get_parameter('width').value/self.res)); self.ny=max(1,int(self.get_parameter('height').value/self.res))
        self.min_points=int(self.get_parameter('min_points_per_cell').value)
        self.percentile=float(self.get_parameter('ground_percentile').value)
        self.max_sensor_age=float(self.get_parameter('max_sensor_age').value)
        self.last_cloud_stamp_ns=None
        self.step=float(self.get_parameter('max_height_step').value); self.lethal=float(self.get_parameter('lethal_height_step').value)
        self.tf_buffer=Buffer(); self.tf_listener=TransformListener(self.tf_buffer,self)
        self.grid_pub=self.create_publisher(OccupancyGrid,'/terrain/local_costmap',1)
        self.elev_pub=self.create_publisher(PointCloud2,'/terrain/elevation_cloud',1)
        self.sub=self.create_subscription(PointCloud2,self.get_parameter('input_topic').value,self.cloud_cb,qos_profile_sensor_data)
        self.last_points=None
        self.timer=self.create_timer(1.0/max(0.1,float(self.get_parameter('publish_rate').value)),self.publish_maps)
        self.get_logger().info('CPU terrain mapper ready; waiting for PointCloud2 and odom TF')

    def cloud_cb(self,msg):
        try:
            stamp=Time.from_msg(msg.header.stamp)
            tf=self.tf_buffer.lookup_transform(self.frame,msg.header.frame_id,stamp)
            cloud=do_transform_cloud(msg,tf)
            self.last_points=self._read_xyz(cloud)
            self.last_cloud_stamp_ns=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        except Exception as exc:
            self.get_logger().warning(f'Cloud transform/parse unavailable: {exc}')

    @staticmethod
    def _read_xyz(msg):
        fields={f.name:f for f in msg.fields}
        if not all(k in fields for k in ('x','y','z')): return np.empty((0,3),dtype=np.float32)
        fx,fy,fz=(fields[k] for k in ('x','y','z'))
        datatype={PointField.FLOAT32:'f',PointField.FLOAT64:'d'}
        if fx.datatype not in datatype or fy.datatype not in datatype or fz.datatype not in datatype: return np.empty((0,3),dtype=np.float32)
        fmt='<' + datatype[fx.datatype] + datatype[fy.datatype] + datatype[fz.datatype]
        size=struct.calcsize(fmt); pts=[]
        for v in range(msg.height):
            base=v*msg.row_step
            for u in range(msg.width):
                off=base+u*msg.point_step
                try: x,y,z=struct.unpack_from(fmt,msg.data,off+fx.offset) if fx.offset==fy.offset==fz.offset else (struct.unpack_from('<'+datatype[fx.datatype],msg.data,off+fx.offset)[0],struct.unpack_from('<'+datatype[fy.datatype],msg.data,off+fy.offset)[0],struct.unpack_from('<'+datatype[fz.datatype],msg.data,off+fz.offset)[0])
                except (struct.error,TypeError): continue
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z): pts.append((x,y,z))
        return np.asarray(pts,dtype=np.float32) if pts else np.empty((0,3),dtype=np.float32)

    def publish_maps(self):
        if self.last_points is None or len(self.last_points)==0: return
        now_msg=self.get_clock().now().to_msg()
        now_ns=now_msg.sec*1000000000+now_msg.nanosec
        if self.last_cloud_stamp_ns is None or (now_ns-self.last_cloud_stamp_ns)>self.max_sensor_age*1e9:
            unknown=np.full((self.ny,self.nx),-1,dtype=np.int8)
            try:
                base_tf=self.tf_buffer.lookup_transform(self.frame, self.get_parameter('base_frame').value, rclpy.time.Time())
                ox=float(base_tf.transform.translation.x)-self.nx*self.res/2
                oy=float(base_tf.transform.translation.y)-self.ny*self.res/2
                self._publish(unknown,ox,oy,now_msg,self.grid_pub)
            except Exception:
                pass
            return
        pts=self.last_points
        # Rolling map centered on the latest cloud's sensor origin projected into output frame.
        try:
            base_tf=self.tf_buffer.lookup_transform(self.frame, self.get_parameter('base_frame').value, rclpy.time.Time())
            cx=float(base_tf.transform.translation.x); cy=float(base_tf.transform.translation.y)
        except Exception as exc:
            self.get_logger().warning(f'Base pose unavailable; cannot center rolling map: {exc}')
            return
        ox=cx-self.nx*self.res/2; oy=cy-self.ny*self.res/2
        ix=np.floor((pts[:,0]-ox)/self.res).astype(np.int32); iy=np.floor((pts[:,1]-oy)/self.res).astype(np.int32)
        valid=(ix>=0)&(ix<self.nx)&(iy>=0)&(iy<self.ny)&(pts[:,2]>-2.0)&(pts[:,2]<3.0)
        ix,iy,z=ix[valid],iy[valid],pts[valid,2]
        cells={}
        for x,y,h in zip(ix,iy,z): cells.setdefault((x,y),[]).append(float(h))
        elev=np.full((self.ny,self.nx),np.nan,dtype=np.float32)
        for (x,y),vals in cells.items():
            if len(vals)>=self.min_points: elev[y,x]=np.percentile(vals,self.percentile)
        cost=np.full((self.ny,self.nx),-1,dtype=np.int8)
        known=np.isfinite(elev)
        for y,x in zip(*np.where(known)):
            neigh=elev[max(0,y-1):min(self.ny,y+2),max(0,x-1):min(self.nx,x+2)]
            vals=neigh[np.isfinite(neigh)]
            if len(vals)<2: cost[y,x]=0; continue
            dh=float(np.max(vals)-np.min(vals))
            cost[y,x]=100 if dh>=self.lethal else (min(99,int(1+98*dh/max(self.step,1e-3))) if dh>self.step else 0)
        now=now_msg
        self._publish(cost,ox,oy,now,self.grid_pub)
        # Metric elevation output: one XYZ point at each observed cell center.
        self._publish_elevation_cloud(elev, known, ox, oy, now)

    def _publish_elevation_cloud(self, elev, known, ox, oy, stamp):
        msg=PointCloud2(); msg.header.frame_id=self.frame; msg.header.stamp=stamp
        msg.height=1; msg.width=int(np.count_nonzero(known)); msg.is_bigendian=False; msg.is_dense=True
        msg.fields=[PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1), PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1), PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1)]
        msg.point_step=12; msg.row_step=msg.point_step*msg.width
        data=bytearray()
        for y,x in zip(*np.where(known)):
            data.extend(struct.pack('<fff', ox+(x+0.5)*self.res, oy+(y+0.5)*self.res, float(elev[y,x])))
        msg.data=bytes(data); self.elev_pub.publish(msg)

    def _publish(self,data,ox,oy,stamp,pub):
        msg=OccupancyGrid(); msg.header.frame_id=self.frame; msg.header.stamp=stamp
        msg.info.resolution=self.res; msg.info.width=self.nx; msg.info.height=self.ny
        msg.info.origin.position.x=float(ox); msg.info.origin.position.y=float(oy); msg.info.origin.orientation.w=1.0
        msg.data=data.reshape(-1).tolist(); pub.publish(msg)

def main(args=None):
    rclpy.init(args=args); node=TerrainMapper()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()
