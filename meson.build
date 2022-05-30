project(
  'pld-managerd',
  'cpp',
  version : '0.1',
  default_options : [
    'warning_level=3',
    'cpp_std=c++20'
  ]
)

deps = [
  dependency('libgpiodcxx', fallback: ['libgpiod', 'gpiodcxx_dep'], default_options: ['bindings=cxx']),
  dependency('systemd'),
  dependency('sdbusplus', fallback: ['sdbusplus', 'sdbusplus_dep']),
]

executable(
  'pld-managerd',
  'src/pld_managerd.cpp',
  dependencies: deps,
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
