#!/usr/bin/ruby

require 'optparse'
require 'fileutils'

include FileUtils

px4_dir="px4dir"

sitl_gazebo_dir = "sitl_gazebo"
firmware_dir = "Firmware"
px4_fname="px4"


#script dir
@root_dir = __dir__ + '/'

#current dir
@current_dir = Dir.pwd + '/'

#options
all_model_names = ["iris", "iris_opt_flow"]
opts = {
  model: "iris",
  num: 1,
  rate: 10000,
  filter: "ekf2",
  workspace: "workspace",
  gazebo: "gazebo",
  catkin_ws: "workspace/catkin_ws",
  rosinstall: "deps.rosinstall",
  base_port: 15010,
  port_step: 10,
  distance: 2,
  firmware_in: "../../../",
  sitl_gazebo_in: "../"
}

#sitl_gazebo
model_opts_open = "<?xml version=\"1.0\" ?>
<options>\n"
model_opts_close = '</options>'


def xspawn(term_name, cmd, debug)
  term = debug ? "xterm -T #{term_name} -hold -e" : ""
  pid = spawn("#{term} #{cmd}", [:out, :err]=>"/dev/null")
  Process.detach(pid)
end

def create_fcu_files(opts,m_num)
  rc_script = @firmware_dir + "posix-configs/SITL/init/#{opts[:filter]}/#{opts[:model]}"
  @rc_file="rcS#{m_num}"

  unless File.exist?(@rc_file)
    mkdir_p "rootfs/fs/microsd"
    mkdir_p "rootfs/eeprom"
    touch "rootfs/eeprom/parameters"

    cp_r @firmware_dir+"ROMFS/px4fmu_common/mixers", "./"

    #generate rc file
    rc1 ||= File.read(rc_script)
    rc = rc1.sub('param set MAV_TYPE',"param set MAV_SYS_ID #{m_num}\nparam set MAV_TYPE")
    rc.sub!('ROMFS/px4fmu_common/','')
    unless opts[:logging]
      rc.sub!(/sdlog2 start.*\n/,'')
      rc.sub!(/logger start.*\n/,'')
    end
    rc.sub!(/.*OPTICAL_FLOW_RAD.*\n/,'') if opts[:model]=="iris"

    rc.sub!(/simulator start -s.*$/,"simulator start -s -u #{@sim_port}")

    rc.gsub!("-r 4000000","-r #{opts[:rate]}")

    rc.gsub!("-u 14556","-u #{@mav_port}")
    rc.sub!("mavlink start -x -u #{@mav_port}","mavlink start -x -u #{@mav_port} -o #{@mav_oport}")

    rc.sub!("-u 14557","-u #{@mav_port2}")
    rc.sub!("-r 250 -s HIGHRES_IMU -u #{@mav_port}", "-r #{opts[:imu_rate]} -s HIGHRES_IMU -u #{@mav_port2}") if opts[:imu_rate]
    rc.sub!("-o 14540","-o #{@mav_oport2}")
    rc.sub!("gpssim start","param set MAV_USEHILGPS 1") if opts[:hil_gps]

    File.open(@rc_file, 'w') { |out| out << rc }
  end
end

#options
op = OptionParser.new do |op|
  op.banner = "Usage: #{__FILE__} [options] [world_file]"

  op.on("-m MODEL", all_model_names, all_model_names.join(", ")) do |m|
    opts[:model] = m
  end

  op.on("--gazebo_model MODEL", "gazebo model name") do |p|
    opts[:gazebo_model] = p
  end

  op.on("-n NUM", Integer, "number of instances") do |n|
    opts[:num] = n
  end

  op.on("-r RATE", Integer, "px4 data rate") do |r|
    opts[:rate] = r
  end

  op.on("--imu_rate IMU_RATE", Integer, "imu rate") do |p|
    opts[:imu_rate] = p
  end

  op.on("--hil_gps", "turn on hil_gps mode") do
    opts[:hil_gps] = true
  end

  op.on("--plugin_lists PATH", "path to mavros pluginlists.yaml") do |p|
    opts[:plugin_lists] = p
  end

  op.on("-f FILTER", "filter") do |o|
    opts[:filter] = o
  end

  op.on("--logging", "turn on logging") do
    opts[:logging] = true
  end

  op.on("--restart", "soft restart") do
    opts[:restart] = true
    puts "restarting ..."
  end

  op.on("--debug", "debug") do
    opts[:debug] = true
  end

  op.on("--nomavros", "without mavros") do
    opts[:nomavros] = true
  end

  op.on("--workspace PATH", "path to workspace") do |p|
    opts[:workspace] = p
  end

  op.on("--gazebo PATH", "path to gazebo resources") do |p|
    opts[:gazebo] = p
  end

  op.on("--catkin_ws PATH", "path to catkin workspace") do |p|
    opts[:catkin_ws] = p
  end

  op.on("--rosinstall PATH", "path to rosinstall file") do |p|
    opts[:rosinstall] = p
  end

  op.on("--base_port PORT", Integer, "base port") do |p|
    opts[:base_port] = p
  end

  op.on("--port_step STEP", Integer, "port step") do |p|
    opts[:port_step] = p
  end

  op.on("--distance DISTANCE", Integer, "distance between models") do |p|
    opts[:distance] = p
  end

  op.on("--firmware_in PATH", "relative path to #{firmware_dir}") do |p|
    opts[:firmware_in] = p
  end

  op.on("--sitl_gazebo_in PATH", "relative path to #{sitl_gazebo_dir}") do |p|
    opts[:sitl_gazebo_in] = p
  end

  op.on("-h", "help") do
    puts op
    exit
  end
end
op.parse!

@firmware_dir = @root_dir + opts[:firmware_in] + firmware_dir + '/'
sitl_gazebo_dir = @root_dir + opts[:sitl_gazebo_in] + sitl_gazebo_dir

opts[:world] = ARGV[0] ? File.expand_path(ARGV[0], @current_dir) : sitl_gazebo_dir + "/worlds/#{opts[:model]}.world"
unless File.exist?(opts[:world])
  puts "#{opts[:world]} not exist"
  exit
end

gazebo_model = opts[:gazebo_model] || opts[:model]

opts[:workspace] = File.expand_path(opts[:workspace], @current_dir) + "/"
opts[:gazebo] = File.expand_path(opts[:gazebo], @current_dir)
opts[:catkin_ws] = File.expand_path(opts[:catkin_ws], @current_dir)
opts[:rosinstall] = File.expand_path(opts[:rosinstall], @current_dir)
opts[:plugin_lists] = File.expand_path(opts[:plugin_lists], @current_dir) if opts[:plugin_lists]

px4_dir = opts[:workspace] + px4_dir + "/"

#init
cd @root_dir

if File.exist?(opts[:rosinstall]) and not Dir.exist?(opts[:catkin_ws])
  system("./install/catkin_prepare.sh #{opts[:catkin_ws]} #{opts[:rosinstall]}")
end

if opts[:restart]
  system("./kill_px4.sh")
else
  system("./kill_sitl.sh")
end
sleep 1

#start

unless File.exist?(px4_dir + px4_fname)
  mkdir_p px4_dir
  cp @firmware_dir + "build/posix_sitl_default/" + px4_fname, px4_dir
end


world_sdf = File.read(opts[:world])

world_name = world_sdf[/<world name="(.*)">/, 1]
world_fname = "#{world_name}.world"
models_opts_fname = "#{world_name}.xml"

world_sdf.sub!(/\n[^<]*<include>[^<]*<uri>model:\/\/iris[^<]*<\/uri>.*?<\/include>/m, "")

model_incs = ""
model_opts = ""
opts[:num].times do |i|
  x=i*opts[:distance]
  m_index=i
  m_num=i+1

  @mav_port = opts[:base_port] + m_index*opts[:port_step]
  @mav_port2 = @mav_port + 1

  @mav_oport = @mav_port + 5
  @mav_oport2 = @mav_port + 6

  @sim_port = @mav_port + 9

  @bridge_port = @mav_port + 2000

  model_name="#{gazebo_model}#{m_num}"

  cd(px4_dir) {
    mkdir_p model_name

    cd(model_name) {
      create_fcu_files(opts,m_num)

      #run px4
      xspawn("px4-#{m_num}", "../#{px4_fname} -d #{@rc_file}", opts[:debug])
    }
  }

  #generate model
  n = "<name>#{model_name}</name>"
  model_incs += "    <include>
      <uri>model://#{gazebo_model}</uri>
      <pose>#{x} 0 0 0 0 0</pose>
      #{n}
    </include>\n" unless world_sdf.include?(n)

  model_opts += "  <model>
    <name>#{model_name}</name>
    <mavlink_udp_port>#{@sim_port}</mavlink_udp_port>\n"
  model_opts += "  </model>\n"

  cd("mavros") {
    sleep 1

    pl="plugin_lists:=#{opts[:plugin_lists]}" if opts[:plugin_lists]
    launch_opts = "fcu_url:=udp://127.0.0.1:#{@mav_oport2}@127.0.0.1:#{@mav_port2} gcs_inport:=#{@bridge_port}"

    xspawn("mavros-#{m_num}", "./roslaunch.sh #{opts[:catkin_ws]} num:=#{m_num} #{pl} #{launch_opts}", opts[:debug])

  } unless opts[:restart] or opts[:nomavros]

end

if opts[:restart]
  system("gz world -o")
else
  File.open(opts[:workspace] + models_opts_fname, 'w') do |out|
    out << model_opts_open
    out << model_opts
    out << model_opts_close
  end

  #run gzserver
  File.open(opts[:workspace] + world_fname, 'w') do |out|
    world_sdf.each_line {|l|
      out << l
      if l =~ /.*<world.*\n/
        out << model_incs
      end
    }

  end

  cd(opts[:workspace]) {
    xspawn("gazebo", "#{@root_dir}gazebo.sh #{world_fname} #{opts[:gazebo]} #{sitl_gazebo_dir}", opts[:debug])
  }
end
