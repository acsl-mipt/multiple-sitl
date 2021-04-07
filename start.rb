#!/usr/bin/ruby

require 'optparse'
require 'fileutils'

include FileUtils

def xspawn(term_name, cmd, debug = false, env = {})
  term = debug ? "xterm -T #{term_name} -hold -e" : ""
  pid = spawn(env, "#{term} #{cmd}", [:out, :err]=>"/dev/null")
  Process.detach(pid)
  pid
end

def create_fcu_files()
  mkdir_p @abs[:workspace_firmware]
  cd(@abs[:workspace_firmware]) {
    for sym in [:firmware_rc, :firmware_vars, :firmware_params, :firmware_mavlink]
      cp @abs[sym], '.'
    end

    File.open(@rels[:firmware_vars], 'a') do |out|
      out.puts "PX4_SIM_MODEL=#{@opts[:firmware_model] || @opts[:gazebo_model]}"
      out.puts "PX4_ESTIMATOR=#{@opts[:firmware_estimator]}"

      out.puts "base_port=$((#{@opts[:ports_base]}+inst*#{@opts[:ports_step]}))"
      out.puts "simulator_opts=\"-#{@opts[:use_tcp] ? 'c' : 'u'} $((base_port+#{@opts[:pd_sim]}))\""

      out.puts "udp_gcs_port_local=$((base_port+#{@opts[:pd_gcs]}))"
      out.puts "udp_offboard_port_local=$((base_port+#{@opts[:pd_offb]}))"
      out.puts "udp_onboard_payload_port_local=$((base_port+#{@opts[:pd_payl]}))"

      out.puts "udp_gcs_port_remote=$((base_port+#{@opts[:pd_gcs_out]}))"
      out.puts "udp_offboard_port_remote=$((base_port+#{@opts[:pd_offb_out]}))"
      out.puts "udp_onboard_payload_port_remote=$((base_port+#{@opts[:pd_payl_out]}))"
    end

    File.open(@rels[:firmware_params], 'a') do |out|
      out.puts "param set SDLOG_MODE -1" unless @opts[:logging]
      #TODO
      out.puts "param set MAV_USEHILGPS 1" if @opts[:hil_gps]
    end
  }
end

def check_expanded_path(file_name, dir = nil, msg = nil)
  p = File.expand_path(file_name, dir)

  unless File.exist?(p)
    puts "#{p} does not exist" + (msg ? ", #{msg}" : "")
    exit
  end

  p
end

def expand_firmware_files()
  for sym in [:firmware_build, :firmware_etc]
    @abs[sym] = check_expanded_path(@rels[sym], @abs[:firmware])
  end

  for sym in [:firmware_bin, :firmware_sg_build]
    @abs[sym] = check_expanded_path(@rels[sym], @abs[:firmware_build])
  end

  def_firmware_initd = File.expand_path(@rels[:firmware_initd], @abs[:firmware_etc])

  for sym in [:firmware_rc, :firmware_vars, :firmware_params, :firmware_mavlink]
    #set by user
    if @abs[:firmware_initd]
      p = File.expand_path(@rels[sym], @abs[:firmware_initd])
      @abs[sym] = p if File.exist?(p)
    end

    #default
    @abs[sym] = check_expanded_path(@rels[sym], def_firmware_initd) unless @abs[sym]
  end
end

def xml_elements(kv)
  s = ""
  kv.each { |k, v| s<<"<#{k}>#{v}</#{k}>" }

  s
end

def check_gazebo_resource(key)
  unless @abs[key]
    a = []
    for sym in [:gazebo, :sitl_gazebo]
      a << @abs[sym]
      ap = File.expand_path(@rels[key], @abs[sym])
      if File.exist?(ap)
        @abs[key] = ap
        break
      end
    end

    unless @abs[key]
      puts @rels[key] + " does not exist in " + a.join(", ")
      exit
    end
  end
end

def expand_and_check()
  #do not check
  for sym in [:workspace, :gazebo, :catkin_ws]
    @abs[sym] = File.expand_path(@opts[sym])
  end

  #check
  for sym in [:firmware, :sitl_gazebo, :plugin_lists, :world_sdf, :firmware_initd, :pose_list]
    @abs[sym] = check_expanded_path(@opts[sym]) if @opts[sym]
  end

  #set defaults if not set
  for sym in [:firmware]
    @abs[sym] = check_expanded_path(@rels[sym], @abs[:home]) unless @abs[sym]
  end

  for sym in [:sitl_gazebo]
    @abs[sym] = check_expanded_path(@rels[sym], @abs[:firmware]) unless @abs[sym]
  end

  #check firmware files
  expand_firmware_files()

  #gazebo resources
  for sym in [:world_sdf, :model_sdf]
    check_gazebo_resource(sym)
  end

  #workspace
  for sym in [:workspace_firmware, :workspace_model_sdf, :workspace_world_sdf, :workspace_model_opts]
    @abs[sym] = File.expand_path(@rels[sym], @abs[:workspace])
  end

end

def load_contents()
  #contents
  @contents = {}
  for sym in [:world_sdf, :model_sdf, :pose_list]
    @contents[sym] = File.read(@abs[sym]) if @abs[sym]
  end

  if @contents[:pose_list]
    @contents[:pose_list] = @contents[:pose_list].split("\n").map{ |l| l.strip.split }
  end
end

def iterate_instances
  ports = {}

  @opts[:n].times { |m_index|
    m_num = m_index+1
    model_name="#{@opts[:gazebo_model]}#{m_num}"

    p = @opts[:ports_base] + m_index*@opts[:ports_step]

    for n in @port_names
      ports[n] = p + @opts["pd_#{n}".to_sym]
    end

    yield m_index, m_num, model_name, ports
  }
end

def start_firmware(m_index, m_num, model_name, ports)
  fw_name = @rels[:firmware_bin].split('/').last

  cd(@abs[:workspace_firmware]) {
    #firmware instance
    mkdir_p model_name
    cd(model_name) {
      #run
      xspawn("#{fw_name}-#{m_num}", "#{@abs[:firmware_bin]} #{@opts[:debug] ? '' : '-d'} -i #{m_index} -s ../#{@rels[:firmware_rc]} #{@abs[:firmware_etc]}", @opts[:debug], {'PATH'=> ENV['PATH'] + ':../'})
    }
  }
end

def wait_process(p_str, dt = 0.2, to_finish = true, timeout = 100000)
  t = 0
  ret = false
  while true
    IO.popen("pgrep -f '#{p_str}'") { |io|
      pids = io.readlines.map { |p| p.to_i }
      if to_finish
        ret = pids.size == 1
      else
        ret = pids.size > 1
      end
    }
    if ret or t>timeout
      break
    end
    sleep dt
    t+=dt
  end

  ret
end

def update_model(tags_values)
  ok = true

  tags_values.each { |k, v|
    if @contents[:model_sdf].sub!(/<#{k}>.+<\/#{k}>/,"<#{k}>#{v}</#{k}>") == nil
      puts("#{k} tag not found in #{@abs[:model_sdf]}")
      ok = false
    end
  }

  ok
end

def gz_env()
  env = {
    'GAZEBO_PLUGIN_PATH'=> 'build',
    'GAZEBO_MODEL_PATH'=> 'models'
  }

  syms = [:gazebo, :sitl_gazebo]
  if @opts[:nospawn]
    syms.insert(0, :workspace)
  end

  env.each { |k,v|
    env[k] = ''
    for sym in syms
      p = @abs[sym] + '/' + v
      if sym == :sitl_gazebo and k == 'GAZEBO_PLUGIN_PATH' and @opts[:sitl_gazebo] == nil
        p = @abs[:firmware_sg_build]
      end

      env[k] += p + ':'
    end
  }
  cmd = @abs[:home] + "/gz_env.sh"


  if @opts[:nospawn]
    @opts[:home_dt] = 3 unless @opts[:home_dt]
    @opts[:home_gps] = ['0', '0', '200'] unless @opts[:home_gps]
  end

  if @opts[:home_dt]
    env['PX4_HOME_DT'] = @opts[:home_dt].to_s
  end

  if @opts[:home_gps]
    env['PX4_HOME_LAT'] = @opts[:home_gps][0]
    env['PX4_HOME_LON'] = @opts[:home_gps][1]
    env['PX4_HOME_ALT'] = @opts[:home_gps][2]
  end

  return env, cmd
end

def start_gazebo()
  env, cmd = gz_env()

  #paths
  mkdir_p @abs[:workspace]

  #sitl_gazebo bug/feature: imu plugin block must be on the last
  m = /(.+)(<plugin.*imu_plugin.*?plugin>)(.*?)(<\/model>.*)/m.match(@contents[:model_sdf])
  if m
    @contents[:model_sdf] = m[1] + m[3] + m[2] + m[4]
  end

  if not update_model(@opts[:go])
    puts("check -g options")
    exit
  end

  if not update_model({@opts[:use_tcp] ? 'mavlink_tcp_port' : 'mavlink_udp_port' => '__MAVLINK_PORT__'})
    puts("add/remove --use_tcp option")
    exit
  end

  world_sdf = @abs[:world_sdf]
  if @opts[:nospawn]
    m_dir = @abs[:workspace] + "/models"
    mkdir_p m_dir
    cp_r(File.dirname(@abs[:model_sdf]), m_dir)

    File.open(@abs[:workspace_model_sdf], 'w') do |out|
      out << @contents[:model_sdf]
    end

    File.open(@abs[:workspace_model_opts], 'w') do |out|
      out << "<?xml version=\"1.0\" ?>\n  <options>\n"
      iterate_instances { |m_index, m_num, model_name, ports|
        port = @opts[:hitl] ? ports[:offb] : ports[:sim]
        out << "  <model name=\"#{model_name}\">" + xml_elements({@opts[:use_tcp] ? 'mavlink_tcp_port' : 'mavlink_udp_port' => port}) + "</model>\n"
      }
      out << "</options>"
    end

    model_incs = ""
    iterate_instances { |m_index, m_num, model_name, ports|
      model_incs += "\n    <include><name>#{model_name}</name><uri>model://#{@opts[:gazebo_model]}</uri><pose>#{poses(m_index).join(' ')}</pose></include>"
    }

    m = /(.+<world .*?>)(.*)/m.match(@contents[:world_sdf])
    if m
      @contents[:world_sdf] = m[1] + model_incs + m[2]
    end

    world_sdf = @abs[:workspace_world_sdf]
    File.open(world_sdf, 'w') do |out|
      out << @contents[:world_sdf]
    end

  end

  if @opts[:gazebo_ros]
    cmd += " rosrun gazebo_ros"
  end

  cd(@abs[:workspace]) {
    xspawn("gazebo", "#{cmd} gazebo --verbose #{world_sdf}", @opts[:debug], env)
  }
end

def gz_model(model_name, add_opts)
  env, cmd = gz_env()

  system(env, "#{cmd} gz model --verbose -m #{model_name} #{add_opts}", @opts[:debug] ? {} : {[:out, :err]=>"/dev/null"})
end


def poses(m_index)
  l = m_index*@opts[:distance]
  r = (@opts[:n]-1)*@opts[:distance]/2.0

  #relative pose
  p = [
    - @opts[:distance],
    l - r,
    0,
    0, 0, 0
  ]

  if @contents[:pose_list]
    pose = @contents[:pose_list][m_index]
    if pose
      pose.each_index { |i|
        p[i] = pose[i].to_f if p[i]
      }
    end
  end

  #absolute pose
  @opts[:ref_point].each_index { |i|
    p[i] += @opts[:ref_point][i]
  }

  p
end

def insert_gz_model(m_index, m_num, model_name, ports)
  env, cmd = gz_env()

  port = @opts[:hitl] ? ports[:offb] : ports[:sim]
  File.open(@abs[:workspace_model_sdf], 'w') do |out|
    out << @contents[:model_sdf].sub('__MAVLINK_PORT__', port.to_s)
  end

  gz_model(model_name, "-f #{@abs[:workspace_model_sdf]} -x #{@opts[:ref_point][0]} -y #{@opts[:ref_point][1]} -z #{@opts[:ref_point][2]}")
end

def move_gz_model(m_index, m_num, model_name, ports)
  env, cmd = gz_env()

  p = poses(m_index)

  o = ['x', 'y', 'z', 'R', 'P', 'Y'].map.with_index {|x, i| "-#{x} #{p[i]}"}

  gz_model(model_name, o.join(' '))
end

def start_mavros(m_index, m_num, model_name, ports)
  args={
    num: m_num,
    fcu_url: "udp://127.0.0.1:#{ports[:offb_out]}@127.0.0.1:#{ports[:offb]}",
    gcs_inport: ports[:gcs_mavros]
  }
  args[:plugin_lists] = @abs[:plugin_lists] if @abs[:plugin_lists]

  launch = "px4_#{@opts[:single] ? 'single' : 'num'}.launch"
  args.each { |k, v| launch<<" #{k}:=#{v}" }

  cd(@abs[:home] + "/mavros") {
    xspawn("mavros-#{m_num}", "./ros_env.sh #{@abs[:catkin_ws]} roslaunch #{launch}", @opts[:debug])
  }
end

####################################

#default options
@opts = {
  n: 1,
  firmware_estimator: "ekf2",
  gazebo_model: "iris",
  go: {},

  ports_base: 15010,
  ports_step: 10,
  pd_gcs: 0,
  pd_gcs_out: 5,
  pd_offb: 1,
  pd_offb_out: 6,
  pd_payl: 2,
  pd_payl_out: 7,
  pd_sim: 9,
  pd_gcs_mavros: 2000,

  distance: 2.2,
  ref_point: [0, 0, 0.1],
  build_label: "default",

  workspace: "workspace",
  gazebo: "gazebo",
  catkin_ws: "workspace/catkin_ws",
}

@port_names = []

#parse options
OptionParser.new do |op|
  op.banner = "Usage: #{__FILE__} [options] [world_file]"

  op.on("-n NUM", Integer, "number of instances")
  op.on("-g PARAM=VALUE", /(.+)=(.+)/, "gazebo model option") { |p, k, v| @opts[:go][k] = v }

  op.on("--firmware_estimator NAME")
  op.on("--firmware_model NAME")
  op.on("--gazebo_model NAME")
  #TODO
  op.on("--imu_rate IMU_RATE", Integer)
  #TODO
  op.on("--hil_gps", "turn on hil_gps mode")

  op.on("--plugin_lists PATH", "path to mavros pluginlists.yaml")
  op.on("--logging", "turn on logging")
  op.on("--debug", "debug mode")
  op.on("--nomavros", "without mavros")


  op.on("--firmware_initd PATH", "path to dir with user defined firmware files ")
  op.on("--workspace PATH", "path to workspace")
  op.on("--gazebo PATH", "path to gazebo resources")
  op.on("--catkin_ws PATH", "path to catkin workspace")

  op.on("--ports_base PORT", Integer)
  op.on("--ports_step STEP", Integer)
  for k,v in @opts
    if k.start_with?('pd_')
      op.on("--#{k} PORT_DELTA", Integer, "port delta from base port")
      @port_names<<k.to_s.delete_prefix('pd_').to_sym
    end
  end

  op.on("--use_tcp", "SITL with tcp exchange")
  op.on("--distance DISTANCE", Integer, "distance between models")
  op.on("--ref_point x,y,z", Array, "gazebo coordinates to insert models (px4 reference point)")
  op.on("--firmware PATH", "path to firmware folder")
  op.on("--sitl_gazebo PATH", "path to sitl_gazebo folder")
  op.on("--build_label NAME", "build label")
  op.on("--single", "single mavros node")
  op.on("--hitl", "HITL mode")
  op.on("--gazebo_ros", "use gazebo_ros")
  op.on("--pose_list PATH", "path to models pose list")
  op.on("--nolockstep", "lockstep disabled")
  op.on("--nospawn", "without spawn")
  op.on("--home_gps x,y,z", Array, "PX4_HOME_LAT, PX4_HOME_LON, PX4_HOME_ALT env variables")
  op.on("--home_dt DT", Float, "PX4_HOME_DT env variable")

  op.on("-h", "--help", "help and show defaults") do
    puts op
    puts "\nDefault options: #{@opts}"

    @opts[:help] = true
  end
end.parse!(into: @opts)
@opts[:world_sdf] = ARGV[0]

@opts[:ref_point].map! { |s| s.to_f }

if @opts[:hitl]
    @opts[:go][:serialEnabled] = 1
    @opts[:go][:hil_mode] = 1
end

@opts[:go][:use_tcp] = @opts[:use_tcp] ? 1 : 0

if @opts[:nolockstep]
  @opts[:go][:enable_lockstep] = 0
  @opts[:build_label] = "nolockstep"
end

#relative paths
@rels = {
  firmware: "../../",
    sitl_gazebo: "Tools/sitl_gazebo",
    firmware_build: "build/px4_sitl_#{@opts[:build_label]}",
      firmware_bin: "bin/px4",
      firmware_sg_build: "build_gazebo",
    firmware_etc: "ROMFS/px4fmu_common",
      firmware_initd: "init.d-posix",
        firmware_rc: "rcS",
        firmware_vars: "rc.vars",
        firmware_params: "rc.params",
        firmware_mavlink: "rc.mavlink",

  world_sdf: "worlds/empty.world",
  model_sdf: "models/#{@opts[:gazebo_model]}/#{@opts[:gazebo_model]}.sdf",

  workspace_model_sdf: "ws.sdf",
  workspace_firmware: "fw",
  workspace_world_sdf: "default.sdf",
  workspace_model_opts: "default.xml",
}

if @opts[:nospawn]
  @rels[:workspace_model_sdf] = @rels[:model_sdf]
end

#script dir
@abs = {
  home: __dir__
}

#expand and check paths
expand_and_check()

if @opts[:help]
  puts "\nDefault absolute paths: #{@abs}"
  puts
  exit
end

load_contents()

#init
cd @abs[:home]

system("./stop.sh")
sleep 1

create_fcu_files()

#start

if @opts[:gazebo_ros] or not @opts[:nomavros]
  xspawn("", @abs[:home] + "/mavros/ros_env.sh #{@abs[:catkin_ws]} roscore")
  sleep 3
end

if @opts[:nospawn]
  iterate_instances { |m_index, m_num, model_name, ports|
    start_firmware(m_index, m_num, model_name, ports) unless @opts[:hitl]
    start_mavros(m_index, m_num, model_name, ports) unless @opts[:nomavros]
  }

  start_gazebo()

  exit
end


if @opts[:nolockstep]

  iterate_instances { |m_index, m_num, model_name, ports|
    start_firmware(m_index, m_num, model_name, ports) unless @opts[:hitl]
    start_mavros(m_index, m_num, model_name, ports) unless @opts[:nomavros]
  }

  start_gazebo()
  sleep 2

  iterate_instances { |m_index, m_num, model_name, ports|
    #start_mavros(m_index, m_num, model_name, ports) unless @opts[:nomavros]
    insert_gz_model(m_index, m_num, model_name, ports)
    #start_firmware(m_index, m_num, model_name, ports) unless @opts[:hitl]
    wait_process("rcS #{m_index}", 0.2)
    sleep 0.5

    move_gz_model(m_index, m_num, model_name, ports)

    sleep 0.5
   }
else

  start_gazebo()
  sleep 2
  if @opts[:gazebo_ros]
    sleep 3
  end

  iterate_instances { |m_index, m_num, model_name, ports|
    start_mavros(m_index, m_num, model_name, ports) unless @opts[:nomavros]
    start_firmware(m_index, m_num, model_name, ports) unless @opts[:hitl]
    wait_process("simulator --instance #{m_index}", 0.1, false)

    insert_gz_model(m_index, m_num, model_name, ports)

    sleep 0.5
    wait_process("rcS #{m_index}", 0.2)

    move_gz_model(m_index, m_num, model_name, ports)

    sleep 0.5
   }

end
