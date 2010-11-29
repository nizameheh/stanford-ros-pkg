
(in-package :asdf)

(defsystem "wifi_sniffer-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "WifiScan" :depends-on ("_package"))
    (:file "_package_WifiScan" :depends-on ("_package"))
    (:file "WifiSniff" :depends-on ("_package"))
    (:file "_package_WifiSniff" :depends-on ("_package"))
    ))
