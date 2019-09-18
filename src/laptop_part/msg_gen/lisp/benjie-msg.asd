
(cl:in-package :asdf)

(defsystem "benjie-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Marcador" :depends-on ("_package_Marcador"))
    (:file "_package_Marcador" :depends-on ("_package"))
    (:file "MarcadorArray" :depends-on ("_package_MarcadorArray"))
    (:file "_package_MarcadorArray" :depends-on ("_package"))
  ))