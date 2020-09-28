#/bin/sh

CA_CNF=openssl.cnf
SERVER_CNF=s_openssl.cnf
CLIENT_CNF=c_openssl.cnf
KEY_SIZE=2048
DURATION=3650
GENERATE_DIR=test_crt
OUT_PATH=./${GENERATE_DIR}
HASH_ALGO=sha256

if [ ! -d ./test_crt ]; then
	mkdir test_crt
fi

# Create Root Private Key
openssl genrsa -out ${OUT_PATH}/ca.key ${KEY_SIZE}
# Create Root Cert
openssl req -new -x509 -days ${DURATION} -key ${OUT_PATH}/ca.key -config ${CA_CNF} -out ${OUT_PATH}/ca.pem -${HASH_ALGO}

# Create Client Private Key
openssl genrsa -out ${OUT_PATH}/cli.key ${KEY_SIZE}
# Create CLient Cert signing request
openssl req -new -key ${OUT_PATH}/cli.key -config ${CLIENT_CNF} -out ${OUT_PATH}/cli.csr -${HASH_ALGO}
# Create Client Cert
openssl x509 -req -days ${DURATION} -in ${OUT_PATH}/cli.csr -CA ${OUT_PATH}/ca.pem -CAkey ${OUT_PATH}/ca.key -CAcreateserial -out ${OUT_PATH}/cli.pem -extensions v3_req -extfile ${CLIENT_CNF} -${HASH_ALGO}

# Create Mqtt Server Private Key
openssl genrsa -out ${OUT_PATH}/serv.key ${KEY_SIZE}
# Create Mqtt Server Cert signing request
openssl req -new -key ${OUT_PATH}/serv.key -config ${SERVER_CNF} -out ${OUT_PATH}/serv.csr -${HASH_ALGO}
# Create Server Cert
openssl x509 -req -days ${DURATION} -in ${OUT_PATH}/serv.csr -CA ${OUT_PATH}/ca.pem -CAkey ${OUT_PATH}/ca.key -CAcreateserial -out ${OUT_PATH}/serv.pem -extensions v3_req -extfile ${SERVER_CNF} -${HASH_ALGO}

echo create ok
