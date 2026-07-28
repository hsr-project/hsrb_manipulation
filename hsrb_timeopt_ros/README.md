このパッケージについて
======================
最短時間制御を用いて，HSRの幾何軌道を受け取り，
リミットを満たす最短時間で再生できる
時間付きの軌道を返す．

開発関係者
==========
* 寺田耕志

インターフェイス
================

* 提供するサービス

- `filter_hsrb_trajectory`(`tmc_manipulation_msgs/FilterJointTrajectory`)

   - Request
     - `trajectory`(`trajectory_msgs/JointTrajectory`): フィルタする軌道．time_from_startは空の状態．poistionのみが意味を持つ．
     - `start_states`(`tmc_manipulation_msgs/RobotState`): 初期状態．
     - `limits`(`tmc_manipulation_msgs/JointLimits`): 各関節のリミット．最終的に上書きされる．
     - `allowed_time`(`duration`): 許容時間[s]．現状利用していない.

   - Response
     - `trajectory`(`trajectory_msgs/JointTrajectory`): フィルタされた軌道．
     - `error_code`(`tmc_manipulation_msgs/ArmNavigationErrorCodes`): エラーコード.
     (SUCCESS=1: 成功, PLANNING_FAILED=-1: 失敗)のどちらかを返す．

* パラメータ

  - `~(joint_name)/velocity` (double, default: 1.0)

      (joint_name)関節の最大速度．[rad/s] or [m/s]

  - `~(joint_name)/acceleration` (double, default: 1.0)

      (joint_name)関節の最大加速度．[rad/s^2] or [m/s^2]

  - `~minimum_dt` (double, default: 0.1)

     移動がない軌道が投げられた場合の再生時間 [s]

  - `~timeopt_resultion` (double, default: 0.2)

     最短時間制御の軌道長パラメータの分割幅． 無次元量．
     分割数をあげれば正確性が増すが計算量が増大する．
     (0.0, 1.0]の範囲にする必要がある．

  - `robot_description` (string)
     ロボットモデル．パースして各種パラメータを取得

  - `~velocity_ratio` (double, default: 1.0)
    速度のリミットに対する割合. dynamic_reconfigure対応. 0.1~1.0で指定．
    全ての関節の速度リミットをまとめて変更する．

  - `~acceleration_ratio` (double, default: 1.0)
    加速度のリミットに対する割合. dynamic_reconfigure対応. 0.1~1.0で指定．
    全ての関節の速度リミットをまとめて変更する．
