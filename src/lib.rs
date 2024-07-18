//! This is a rust port of the [ROS tf library](http://wiki.ros.org/tf). It is intended for being used in robots to help keep track of
//! multiple coordinate frames and is part of a larger suite of rust libraries that provide support for various robotics related functionality.
//!
//! Example usage:
//! ```ignore
//! fn main() {
//!     rosrust::init("listener");
//!     let listener = TfListener::new();
//!
//!     let rate = rosrust::rate(1.0);
//!     while rosrust::is_ok() {
//!         let tf = listener.lookup_transform("camera", "base_link", ros::Time::now());
//!         println!("{:?}", tf);
//!         rate.sleep();
//!     }
//! }
//!```
#[allow(unused_imports)]
use rosrust_msg::geometry_msgs::{Transform, TransformStamped, Quaternion, Vector3};
use rosrust_msg::std_msgs::Header;
use rosrust_msg::tf2_msgs::TFMessage;
use std::cmp::Ordering;
use std::collections::HashMap;
use std::collections::HashSet;
use std::collections::VecDeque;
use std::sync::{Arc, RwLock};

pub mod transforms;

#[derive(Clone, Debug)]
struct OrderedTF {
    tf: TransformStamped,
}

impl PartialEq for OrderedTF {
    fn eq(&self, other: &Self) -> bool {
        self.tf.header.stamp == other.tf.header.stamp
    }
}

impl Eq for OrderedTF {}

impl Ord for OrderedTF {
    fn cmp(&self, other: &Self) -> Ordering {
        self.tf.header.stamp.cmp(&other.tf.header.stamp)
    }
}

impl PartialOrd for OrderedTF {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.tf.header.stamp.cmp(&other.tf.header.stamp))
    }
}

/// Calculates the inverse of a ros transform

/// Enumerates the different types of errors
#[derive(Clone, Debug, PartialEq)]
pub enum TfError {
    /// Error due to looking up too far in the past. I.E the information is no longer available in the TF Cache.
    AttemptedLookupInPast,
    /// Error due ti the transform not yet being available.
    AttemptedLookUpInFuture,
    /// There is no path between the from and to frame.
    CouldNotFindTransform,
    /// In the event that a write is simultaneously happening with a read of the same tf buffer
    CouldNotAcquireLock,
}

fn get_nanos(dur: rosrust::Duration) -> i64 {
    i64::from(dur.sec) * 1_000_000_000 + i64::from(dur.nsec)
}

fn to_transform_stamped(
    tf: Transform,
    from: std::string::String,
    to: std::string::String,
    time: rosrust::Time,
) -> TransformStamped {
    TransformStamped {
        header: Header {
            frame_id: from,
            stamp: time,
            seq: 1u32,
        },
        child_frame_id: to,
        transform: tf,
    }
}

#[derive(Clone, Debug)]
struct TfIndividualTransformChain {
    buffer_size: usize,
    static_tf: bool,
    //TODO:  Implement a circular buffer. Current method is slowww.
    transform_chain: Vec<OrderedTF>,
}

impl TfIndividualTransformChain {
    pub fn new(static_tf: bool) -> Self {
        return TfIndividualTransformChain {
            buffer_size: 100,
            transform_chain: Vec::new(),
            static_tf: static_tf,
        };
    }

    pub fn add_to_buffer(&mut self, msg: TransformStamped) {
        let res = self
            .transform_chain
            .binary_search(&OrderedTF { tf: msg.clone() });

        match res {
            Ok(x) => self.transform_chain.insert(x, OrderedTF { tf: msg }),
            Err(x) => self.transform_chain.insert(x, OrderedTF { tf: msg }),
        }

        if self.transform_chain.len() > self.buffer_size {
            self.transform_chain.remove(0);
        }
    }

    pub fn get_closest_transform(&self, time: rosrust::Time) -> Result<TransformStamped, TfError> {
        if self.static_tf {
            return Ok(self
                .transform_chain
                .get(self.transform_chain.len() - 1)
                .unwrap()
                .tf
                .clone());
        }

        let mut res = TransformStamped::default();
        res.header.stamp = time;
        res.transform.rotation.w = 1f64;

        let res = self.transform_chain.binary_search(&OrderedTF { tf: res });

        match res {
            Ok(x) => return Ok(self.transform_chain.get(x).unwrap().tf.clone()),
            Err(x) => {
                if x == 0 {
                    return Err(TfError::AttemptedLookupInPast);
                }
                if x >= self.transform_chain.len() {
                    return Err(TfError::AttemptedLookUpInFuture);
                }
                let tf1 = self
                    .transform_chain
                    .get(x - 1)
                    .unwrap()
                    .clone()
                    .tf
                    .transform;
                let tf2 = self.transform_chain.get(x).unwrap().clone().tf.transform;
                let time1 = self.transform_chain.get(x - 1).unwrap().tf.header.stamp;
                let time2 = self.transform_chain.get(x).unwrap().tf.header.stamp;
                let header = self.transform_chain.get(x).unwrap().tf.header.clone();
                let child_frame = self
                    .transform_chain
                    .get(x)
                    .unwrap()
                    .tf
                    .child_frame_id
                    .clone();
                let total_duration = get_nanos(time2 - time1) as f64;
                let desired_duration = get_nanos(time - time1) as f64;
                let weight = 1.0 - desired_duration / total_duration;
                let final_tf = transforms::interpolate(tf1, tf2, weight);
                let ros_msg = to_transform_stamped(final_tf, header.frame_id, child_frame, time);
                Ok(ros_msg)
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Node {
    pub parent: String,
    #[allow(dead_code)]
    pub broadcaster: String,
    pub rate: f64,
    pub most_recent_transform: f64,
    pub oldest_transform: f64,
    #[allow(dead_code)]
    pub buffer_length: f64,
}

#[derive(Clone, Debug, Hash)]
struct TfGraphNode {
    child: String,
    parent: String,
}

impl PartialEq for TfGraphNode {
    fn eq(&self, other: &Self) -> bool {
        self.child == other.child && self.parent == other.parent
    }
}

impl Eq for TfGraphNode {}

#[derive(Clone, Debug)]
pub struct TfBuffer {
    child_transform_index: HashMap<String, HashSet<String>>,
    transform_data: HashMap<TfGraphNode, TfIndividualTransformChain>,
}

fn strip_leading_slash(transform: &mut TransformStamped) {
    if transform.child_frame_id.to_string().starts_with('/') {
        transform.child_frame_id.remove(0);
    }
    if transform.header.frame_id.to_string().starts_with('/') {
        transform.header.frame_id.remove(0);
    }
}

impl TfBuffer {
    fn new() -> Self {
        TfBuffer {
            child_transform_index: HashMap::new(),
            transform_data: HashMap::new(),
        }
    }

    fn handle_incoming_transforms(&mut self, transforms: TFMessage, static_tf: bool) {
        for mut transform in transforms.transforms {
            strip_leading_slash(&mut transform);
            self.add_transform(&transform, static_tf);
        }
    }
    

    fn creates_cycle(&self, parent: &String, child: &String) -> bool {
        let mut visited = HashSet::new();
        let mut stack = VecDeque::new();
        stack.push_back(child.clone());

        while let Some(node) = stack.pop_back() {
            if &node == parent {
                return true;
            }
            if visited.contains(&node) {
                continue;
            }
            visited.insert(node.clone());

            if let Some(children) = self.child_transform_index.get(&node) {
                for child in children {
                    if !visited.contains(child) {
                        stack.push_back(child.clone());
                    }
                }
            }
        }
        false
    }

    fn add_transform(&mut self, transform: &TransformStamped, static_tf: bool) {
        if !self.creates_cycle(&transform.header.frame_id.clone(), &transform.child_frame_id) {
            if self
                .child_transform_index
                .contains_key(&transform.header.frame_id)
            {
                let res = self
                    .child_transform_index
                    .get_mut(&transform.header.frame_id.clone())
                    .unwrap();
                res.insert(transform.child_frame_id.clone());
            } else {
                self.child_transform_index
                    .insert(transform.header.frame_id.clone(), HashSet::new());
                let res = self
                    .child_transform_index
                    .get_mut(&transform.header.frame_id.clone())
                    .unwrap();
                res.insert(transform.child_frame_id.clone());
            }
        } else {
            return
        }

        let key = TfGraphNode {
            child: transform.child_frame_id.clone(),
            parent: transform.header.frame_id.clone(),
        };

        if self.transform_data.contains_key(&key) {
            let data = self.transform_data.get_mut(&key).unwrap();
            data.add_to_buffer(transform.clone());
        } else {
            let mut data = TfIndividualTransformChain::new(static_tf);
            data.add_to_buffer(transform.clone());
            self.transform_data.insert(key, data);
        }
    }
    
    fn create_child_to_parent_map(&self) -> HashMap<String, String> {
        let mut child_to_parent: HashMap<String, String> = HashMap::new();
        
        for (parent, children) in self.child_transform_index.iter() {
            for child in children {
                child_to_parent.insert(child.clone(), parent.clone());
            }
        }
        
        child_to_parent
    }

    fn retrieve_transform_path(&self, source_frame_id: String, target_frame_id: String) -> Result<(Vec<String>, Vec<String>), TfError> {
        let child_to_parent = self.create_child_to_parent_map();

        let trace_to_root_or_goal = |node: String, goal: Option<String>,child_to_parent: &HashMap<String, String>| -> Result<Vec<String>, TfError> {
            let mut path = vec![];
            let mut current = node;

            while let Some(parent) = child_to_parent.get(&current) {
                path.push(current.clone());
                if goal.clone().is_some() && current == goal.clone().unwrap(){
                    path.reverse();
                    return Ok(path);
                }
                current = parent.clone();
            }
            path.push(current.clone()); // Add the last node (root)
            path.reverse();
            Ok(path)
        };

        let path_from_source = trace_to_root_or_goal(source_frame_id.clone(), Some(target_frame_id.clone()), &child_to_parent)?;
        let path_to_target = trace_to_root_or_goal(target_frame_id.clone(), Some(source_frame_id.clone()), &child_to_parent)?;
        
        // We are in a same subtree
        if let Some(pos) = path_to_target.iter().position(|x| x == &source_frame_id) {
            return Ok((Vec::new(), path_to_target[pos + 1..].to_vec()));
        } 
        if let Some(pos) = path_from_source.iter().position(|x| x == &target_frame_id) {
            let mut ans = path_from_source[pos..].to_vec();
            ans.reverse();
            return Ok((ans[1..].to_vec(), Vec::new()));
        }

        let mut lowest_common_ancestor = path_from_source.iter()
            .zip(&path_to_target)
            .take_while(|(a, b)| a == b)
            .count();

        if lowest_common_ancestor == 0 {
            return Err(TfError::CouldNotFindTransform);
        }
        lowest_common_ancestor = lowest_common_ancestor - 1;

        // Concatenate paths: source_frame_id -> LCA -> target_frame_id
        let mut final_path_to_target = Vec::new();
        let mut final_path_from_source = Vec::new();
        for i in (lowest_common_ancestor..path_from_source.len()-1).rev() {
            final_path_from_source.push(path_from_source[i].clone());
        }

        // From the LCA to 'target_frame_id', exclusive of the LCA
        for i in lowest_common_ancestor+1..path_to_target.len() {
            final_path_to_target.push(path_to_target[i].clone());
        }

        Ok((final_path_from_source, final_path_to_target))
    }

    /// Looks up a transform within the tree at a given time.
    pub fn lookup_transform(
        &self,
        from: &str,
        to: &str,
        time: rosrust::Time,
    ) -> Result<TransformStamped, TfError> {
        let path = self.retrieve_transform_path(from.to_string(), to.to_string())?;
        let mut tflist = Vec::new();
        let mut prev_frame_id = from;
    
        let (path_from, path_to) = path;
    
        for (index, _) in path_from.iter().chain(&path_to).enumerate() {
            let inverse_needed = index < path_from.len();

            let (parent, child, new_prev) = if inverse_needed {
                (path_from[index].as_str(), prev_frame_id, path_from[index].as_str())
            } else {
                (prev_frame_id, path_to[index - path_from.len()].as_str(), path_to[index - path_from.len()].as_str())
            };
            prev_frame_id = new_prev;

            let node = TfGraphNode {
                parent: parent.to_string(),
                child: child.to_string(),
            };
    
            let transform = self.transform_data.get(&node)
                .ok_or_else(|| TfError::CouldNotFindTransform)      
                .and_then(|tc| tc.get_closest_transform(time))?;
    
            let transform = if inverse_needed {
                transforms::get_inverse(&transform)
            } else {
                transform
            };
    
            tflist.push(transform.transform);
        }
    
        let final_transform = transforms::chain_transforms(&tflist);
    
        Ok(TransformStamped {
            child_frame_id: to.to_string(),
            header: Header {
                frame_id: from.to_string(),
                stamp: time,
                seq: 1,
            },
            transform: final_transform,
        })
    }

    fn lookup_transform_with_time_travel(
        &self,
        to: &str,
        time2: rosrust::Time,
        from: &str,
        time1: rosrust::Time,
        fixed_frame: &str,
    ) -> Result<TransformStamped, TfError> {
        let tf1 = self.lookup_transform(from, fixed_frame, time1);
        let tf2 = self.lookup_transform(to, fixed_frame, time2);
        match tf1 {
            Err(x) => return Err(x),
            Ok(_) => {}
        }
        match tf2 {
            Err(x) => return Err(x),
            Ok(_) => {}
        }
        let transforms = transforms::get_inverse(&tf1.unwrap());
        let result =
            transforms::chain_transforms(&vec![tf2.unwrap().transform, transforms.transform]);
        Ok(to_transform_stamped(
            result,
            from.to_string(),
            to.to_string(),
            time1,
        ))
    }

    pub fn all_frames_as_map(&self)->HashMap<String, Node>{
        let child_to_parent = self.create_child_to_parent_map();

        let mut frames: HashMap<String, Node> = HashMap::new();
        for (child, parent) in child_to_parent {
            let node = TfGraphNode {
                parent: parent.to_string(),
                child: child.to_string(),
            };
            let transform = self.transform_data.get(&node)
                .ok_or_else(|| TfError::CouldNotFindTransform)      
                .and_then(|tc| tc.get_closest_transform(rosrust::Time::new()))
                .unwrap();

            let time = transform.header.stamp.sec as f64 + (transform.header.stamp.nsec as f64 / 1_000_000_000.0);
            
            // TODO some fields are placeholder values here
            // such as rate, broadcaster, oldest_transform, buffer_length
            frames.insert(
                child,
                Node {
                    parent: parent,
                    broadcaster: "unknown_publisher".to_string(),
                    rate: 0.0,
                    most_recent_transform: time,
                    oldest_transform: time,
                    buffer_length: 0.0,
                }
            );
        }

        return frames;

    }
}

#[cfg(test)]
mod test {
    use super::*;

    fn create_transform(frame_id: &str, child_frame_id: &str, x: f64, y: f64, z: f64, w: f64, tx: f64, ty: f64, tz: f64, time: f64) -> TransformStamped {
        let secs = time.floor() as u32;
        let nsecs = ((time - ((time.floor() as i64) as f64)) * 1E9) as u32;

        TransformStamped {
            child_frame_id: child_frame_id.to_string(),
            header: Header {
                frame_id: frame_id.to_string(),
                stamp: rosrust::Time { sec: secs, nsec: nsecs },
                seq: 1,
            },
            transform: Transform {
                rotation: Quaternion { x, y, z, w },
                translation: Vector3 { x: tx, y: ty, z: tz },
            },
        }
    }

    /// Final TF Tree
    ///
    /// world/
    /// ├─ item/
    /// │  ├─ item_link_1/
    /// │  │  ├─ item_link_2/
    /// ├─ base_link(starting at (0,0,0) and progressing at (0,t,0) where t is time in seconds)/
    /// │  ├─ camera/
    /// │  ├─ sensor_base/
    /// │     ├─ sensor_base/
    /// │  ├─ f_r_wheel/
    /// │  ├─ r_r_wheel/
    /// │  ├─ r_l_wheel/
    /// │  ├─ f_l_wheel/
    fn build_test_tree(buffer: &mut TfBuffer, time: f64) {
        let world_to_item = create_transform("world", "item", 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, time);
        buffer.add_transform(&world_to_item, true);

        let item_to_item_link1 = create_transform("item", "item_link_1", 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, time);
        buffer.add_transform(&item_to_item_link1, true);

        let item_link1_to_item_link2 = create_transform("item_link_1", "item_link_2", 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, time);
        buffer.add_transform(&item_link1_to_item_link2, true);

        let world_to_base_link = create_transform("world", "base_link", 0.0, 0.0, 0.0, 1.0, 0.0, time, 0.0, time);
        buffer.add_transform(&world_to_base_link, false);

        let base_link_to_camera = create_transform("base_link", "camera", 0.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0, time);
        buffer.add_transform(&base_link_to_camera, true);


        let base_link_to_f_r_wheel = create_transform("base_link", "f_r_wheel", 0.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0, time);
        buffer.add_transform(&base_link_to_f_r_wheel, true);
        let base_link_to_f_l_wheel = create_transform("base_link", "f_l_wheel", 0.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0, time);
        buffer.add_transform(&base_link_to_f_l_wheel, true);
        let base_link_to_r_r_wheel = create_transform("base_link", "r_r_wheel", 0.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0, time);
        buffer.add_transform(&base_link_to_r_r_wheel, true);
        let base_link_to_r_l_wheel = create_transform("base_link", "r_l_wheel", 0.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0, time);
        buffer.add_transform(&base_link_to_r_l_wheel, true);

        let base_link_to_sensor_base = create_transform("base_link", "sensor_base", 0.707, 0.0, 0.0, 0.707, 0.5, 0.0, 0.0, time);
        buffer.add_transform(&base_link_to_sensor_base, true);
        let sensor_base_to_sensor = create_transform("sensor_base", "sensor", -0.707, 0.0, 0.0, 0.707, 0.0, 0.0, 0.0, time);
        buffer.add_transform(&sensor_base_to_sensor, true);
    }
    /// world/
    /// ├─ item/
    fn build_small_test_tree(buffer: &mut TfBuffer, time: f64) {
        let world_to_item = create_transform("world", "item", 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, time);
        buffer.add_transform(&world_to_item, true);
    }
    
    #[test]
    fn test_cycle() {
        let time = 0f64;
        let mut buffer = TfBuffer::new();
        let world_to_item = create_transform("world", "item", 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, time);
        buffer.add_transform(&world_to_item, true);

        let item_to_item2 = create_transform("item", "item_2", 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, time);
        buffer.add_transform(&item_to_item2, true);

        let item_to_item3 = create_transform("item", "item_3", 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, time);
        buffer.add_transform(&item_to_item3, true);

        assert_eq!(buffer.child_transform_index.keys().len(), 2);
        assert_eq!(buffer.child_transform_index["item"].len(), 2);

        let item_to_world = create_transform("item", "world", 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, time);
        buffer.add_transform(&item_to_world, true);

        let item3_to_world = create_transform("item_3", "world", 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, time);
        buffer.add_transform(&item3_to_world, true);

        assert_eq!(buffer.child_transform_index.keys().len(), 2);
        assert_eq!(buffer.child_transform_index["item"].len(), 2);
    }

    #[test]
    fn test_get_all_frames() {
        let time = 0f64;
        let mut tf_buffer = TfBuffer::new();
        build_test_tree(&mut tf_buffer, time);

        let frames = tf_buffer.all_frames_as_map();
        let nodes_data = vec![
            ("r_r_wheel", "base_link"),
            ("sensor_base", "base_link"),
            ("camera", "base_link"),
            ("item", "world"),
            ("r_l_wheel", "base_link"),
            ("sensor", "sensor_base"),
            ("item_link_1", "item"),
            ("item_link_2", "item_link_1"),
            ("f_l_wheel", "base_link"),
            ("base_link", "world"),
            ("f_r_wheel", "base_link"),
        ];

        let expected: HashMap<String, Node> = nodes_data
            .into_iter()
            .map(|(name, parent)| {
                (
                    name.to_string(),
                    Node {
                        parent: parent.to_string(),
                        broadcaster: "unknown_publisher".to_string(),
                        rate: 0.0,
                        most_recent_transform: time,
                        oldest_transform: time,
                        buffer_length: 0.0,
                    },
                )
            })
            .collect();
        
        assert_eq!(frames, expected);
    }
    #[test]
    fn test_basic_tf_lookup() {
        let time = 0f64;
        let mut tf_buffer = TfBuffer::new();
        build_test_tree(&mut tf_buffer, time);

        let test_cases = [
            ("r_l_wheel", "world", 0.0, 0.0, 0.0, 1.0, -0.5, 0.0, 0.0),
            ("r_l_wheel", "item_link_1", 0.0, 0.0, 0.0, 1.0, 1.5, 0.0, 0.0),
            ("item_link_2", "item", 0.0, 0.0, 0.0, 1.0, -2.0, 0.0, 0.0),
            ("item", "item", 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
            ("camera", "f_l_wheel", 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
            ("world", "sensor_base", 0.707, 0.0, 0.0, 0.707, 0.5, 0.0, 0.0),
            ("world", "sensor", 0.0, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0),
        ];

        for (from, to, x, y, z, w, tx, ty, tz) in test_cases.iter() {
            let result = tf_buffer.lookup_transform(from, to, rosrust::Time { sec: 0, nsec: 0 });
            let expected = create_transform(from, to, *x, *y, *z, *w, *tx, *ty, *tz, time);
            assert_approx_eq(result.unwrap(), expected );
        }

        tf_buffer = TfBuffer::new();
        build_small_test_tree(&mut tf_buffer, time);
        let test_cases = [
            ("world", "item", 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0)
        ];
        for (from, to, x, y, z, w, tx, ty, tz) in test_cases.iter() {
            let result = tf_buffer.lookup_transform(from, to, rosrust::Time { sec: 0, nsec: 0 });
            let expected = create_transform(from, to, *x, *y, *z, *w, *tx, *ty, *tz, time);
            assert_approx_eq(result.unwrap(), expected );
        }
        let result = tf_buffer.lookup_transform("void1", "void2", rosrust::Time { sec: 0, nsec: 0 });
        assert_eq!(result, Err(TfError::CouldNotFindTransform));

        tf_buffer = TfBuffer::new();
        let result = tf_buffer.lookup_transform("void1", "void2", rosrust::Time { sec: 0, nsec: 0 });
        assert_eq!(result, Err(TfError::CouldNotFindTransform));
    }

    /// Tests an interpolated lookup.
    #[test]
    fn test_basic_tf_interpolation() {
        let mut tf_buffer = TfBuffer::new();
        build_test_tree(&mut tf_buffer, 0f64);
        build_test_tree(&mut tf_buffer, 1f64);
        let res = tf_buffer.lookup_transform(
            "camera",
            "item",
            rosrust::Time {
                sec: 0,
                nsec: 700_000_000,
            },
        );
        let expected = TransformStamped {
            child_frame_id: "item".to_string(),
            header: Header {
                frame_id: "camera".to_string(),
                stamp: rosrust::Time {
                    sec: 0,
                    nsec: 700_000_000,
                },
                seq: 1,
            },
            transform: Transform {
                rotation: Quaternion {
                    x: 0f64,
                    y: 0f64,
                    z: 0f64,
                    w: 1f64,
                },
                translation: Vector3 {
                    x: 0.5f64,
                    y: -0.7f64,
                    z: 0f64,
                },
            },
        };
        assert_eq!(res.unwrap(), expected);
    }

    /// Tests an interpolated lookup.
    #[test]
    fn test_basic_tf_timetravel() {
        let mut tf_buffer = TfBuffer::new();
        build_test_tree(&mut tf_buffer, 0f64);
        build_test_tree(&mut tf_buffer, 1f64);
        let res = tf_buffer.lookup_transform_with_time_travel(
            "camera",
            rosrust::Time {
                sec: 0,
                nsec: 400_000_000,
            },
            "camera",
            rosrust::Time {
                sec: 0,
                nsec: 700_000_000,
            },
            "item",
        );
        let expected = TransformStamped {
            child_frame_id: "camera".to_string(),
            header: Header {
                frame_id: "camera".to_string(),
                stamp: rosrust::Time {
                    sec: 0,
                    nsec: 700_000_000,
                },
                seq: 1,
            },
            transform: Transform {
                rotation: Quaternion {
                    x: 0f64,
                    y: 0f64,
                    z: 0f64,
                    w: 1f64,
                },
                translation: Vector3 {
                    x: 0f64,
                    y: 0.3f64,
                    z: 0f64,
                },
            },
        };
        assert_approx_eq(res.unwrap(), expected);
    }

    fn assert_approx_eq(msg1: TransformStamped, msg2: TransformStamped) {
        assert_eq!(msg1.header, msg2.header);
        assert_eq!(msg1.child_frame_id, msg2.child_frame_id);

        assert!((msg1.transform.rotation.x - msg2.transform.rotation.x).abs() < 1e-3, "left {} right {}", msg1.transform.rotation.x, msg2.transform.rotation.x);
        assert!((msg1.transform.rotation.y - msg2.transform.rotation.y).abs() < 1e-3, "left {} right {}", msg1.transform.rotation.y, msg2.transform.rotation.y);
        assert!((msg1.transform.rotation.z - msg2.transform.rotation.z).abs() < 1e-3, "left {} right {}", msg1.transform.rotation.z, msg2.transform.rotation.z);
        assert!((msg1.transform.rotation.w - msg2.transform.rotation.w).abs() < 1e-3, "left {} right {}", msg1.transform.rotation.w, msg2.transform.rotation.w);

        assert!((msg1.transform.translation.x - msg2.transform.translation.x).abs() < 1e-4, "left {} right {}", msg1.transform.translation.x, msg2.transform.translation.x);
        assert!((msg1.transform.translation.y - msg2.transform.translation.y).abs() < 1e-4, "left {} right {}", msg1.transform.translation.y, msg2.transform.translation.y);
        assert!((msg1.transform.translation.z - msg2.transform.translation.z).abs() < 1e-4, "left {} right {}", msg1.transform.translation.z, msg2.transform.translation.z);
    }
}

///This struct tries to be the same as the C++ version of `TransformListener`. Use this struct to lookup transforms.
///
/// Example usage:
///
/// ```ignore
/// fn main() {
///     rosrust::init("listener");
///     let listener = TfListener::new();
///
///     let rate = rosrust::rate(1.0);
///     while rosrust::is_ok() {
///         let tf = listener.lookup_transform("camera", "base_link", ros::Time::now());
///         println!("{:?}", tf);
///         rate.sleep();
///     }
/// }
/// ```
/// Do note that unlike the C++ variant of the TfListener, only one TfListener can be created at a time. Like its C++ counterpart,
/// it must be scoped to exist through the lifetime of the program. One way to do this is using an `Arc` or `RwLock`.
pub struct TfListener {
    pub buffer: Arc<RwLock<TfBuffer>>,
    _static_subscriber: rosrust::Subscriber,
    _dynamic_subscriber: rosrust::Subscriber,
}

impl TfListener {
    /// Create a new TfListener
    pub fn new() -> Self {
        let buff = RwLock::new(TfBuffer::new());
        let arc = Arc::new(buff);
        let r1 = arc.clone();
        let _subscriber_tf = rosrust::subscribe("tf", 100, move |v: TFMessage| {
            r1.write().unwrap().handle_incoming_transforms(v, true);
        })
        .unwrap();

        let r2 = arc.clone();
        let _subscriber_tf_static = rosrust::subscribe("tf_static", 100, move |v: TFMessage| {
            r2.write().unwrap().handle_incoming_transforms(v, true);
        })
        .unwrap();

        TfListener {
            buffer: arc.clone(),
            _static_subscriber: _subscriber_tf_static,
            _dynamic_subscriber: _subscriber_tf,
        }
    }

    /// Looks up a transform within the tree at a given time.
    pub fn lookup_transform(
        &self,
        from: &str,
        to: &str,
        time: rosrust::Time,
    ) -> Result<TransformStamped, TfError> {
        self.buffer.read().unwrap().lookup_transform(from, to, time)
    }

    /// Looks up a transform within the tree at a given time.
    pub fn lookup_transform_with_time_travel(
        &self,
        from: &str,
        time1: rosrust::Time,
        to: &str,
        time2: rosrust::Time,
        fixed_frame: &str,
    ) -> Result<TransformStamped, TfError> {
        self.buffer
            .read()
            .unwrap()
            .lookup_transform_with_time_travel(from, time1, to, time2, fixed_frame)
    }
}
