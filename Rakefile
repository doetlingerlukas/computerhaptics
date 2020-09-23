BAUD = 57600
SERIAL_PORT = 'COM3'
SKETCH = '.\hapvol-handle\hapvol-handle.ino'

task :setup do
  sh 'arduino-cli', 'core', 'install', 'arduino:avr'
end

task :compile => :setup do
  sh 'arduino-cli', 'compile', '--fqbn', 'arduino:avr:uno', SKETCH
end

task :flash => :compile do
  sh 'arduino-cli', 'upload', '-p', SERIAL_PORT, '--fqbn', 'arduino:avr:uno', SKETCH
end

task :clean do
  sh 'rmdir', '/s', '/q', "#{File.dirname(SKETCH)}\\build"
end

task :default => :flash