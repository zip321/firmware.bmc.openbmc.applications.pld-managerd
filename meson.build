project(
  'pld-managerd',
  'cpp',
  version : '0.1',
  default_options : [
    'warning_level=3',
    'cpp_std=c++20'
  ]
)

executable(
  'pld-managerd',
  'src/pld_managerd.cpp',
  install : true,
  install_dir: get_option('bindir')
)

systemd = dependency('systemd')
conf_data = configuration_data()
conf_data.set('bindir', get_option('prefix') / get_option('bindir'))
configure_file(
  input: 'pld-manager.service.in',
  output: 'pld-manager.service',
  configuration: conf_data,
  install: true,
  install_dir: systemd.get_pkgconfig_variable('systemdsystemunitdir'))
