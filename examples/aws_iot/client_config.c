// AWS IoT client endpoint
const char *client_endpoint = "a1qclp14lza6x4.iot.eu-west-1.amazonaws.com";

// AWS IoT device certificate (ECC)
const char *client_cert =
"-----BEGIN CERTIFICATE-----\r\n"
"MIICqTCCAZGgAwIBAgITVRtho5pXp6PMiwhalLTtc149MTANBgkqhkiG9w0BAQsF\r\n"
"ADBNMUswSQYDVQQLDEJBbWF6b24gV2ViIFNlcnZpY2VzIE89QW1hem9uLmNvbSBJ\r\n"
"bmMuIEw9U2VhdHRsZSBTVD1XYXNoaW5ndG9uIEM9VVMwHhcNMTYxMTA1MDQxNzA3\r\n"
"WhcNNDkxMjMxMjM1OTU5WjA6MQswCQYDVQQGEwJBVTETMBEGA1UECAwKU29tZS1T\r\n"
"dGF0ZTEWMBQGA1UECgwNVEFEZXZlbG9wbWVudDBZMBMGByqGSM49AgEGCCqGSM49\r\n"
"AwEHA0IABPp+EUmrRppeu6c8oOplQIXqtibkqGcn9YIRZ9YZZQyxqajv1Hk80yJK\r\n"
"vBmL1Cjg1UmdRitQvKysqwCK5AYgke2jYDBeMB8GA1UdIwQYMBaAFIckozp14uie\r\n"
"cHHADYgpbo195XW1MB0GA1UdDgQWBBSW5/gGIjgw19OWufTHnhqllKFltDAMBgNV\r\n"
"HRMBAf8EAjAAMA4GA1UdDwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAQNwQ\r\n"
"uTsNlNL+2D5tbE2nf0mEwHKT4g/2UKkHaDtLY9QWcmXoBCQwmn0vA/kYCstyIBB5\r\n"
"YJUcrwn/otgfQZHmPLyJ/Iiy44T6ZGLMxes1XXHhrK3rz7BmlfPaatFBlJlaz43Y\r\n"
"N1j7MUGUY/Kxbb2VAFnjcymnzOmKTzBxM53C1yoJWI4aHZj/anGGY109phs0SrKp\r\n"
"g9Cz4PNr0rfO5jguuDp+ZLpzyDaAP/krzf+ej0gNzwmSqHPtG9FiU/i6oE5xYvbq\r\n"
"BL9WgocedgUtbSdaEfUNXriqSIh4BcrFSjcJKX1yE2SLMIiFu6VxTDs6fRZQ+ChH\r\n"
"t/BnBGz//NfhdAr9xA==\r\n"
"-----END CERTIFICATE-----\r\n";

// AWS IoT device private key (ECC)
const char *client_key = 
"-----BEGIN EC PARAMETERS-----\r\n"
"BggqhkjOPQMBBw==\r\n"
"-----END EC PARAMETERS-----\r\n"
"-----BEGIN EC PRIVATE KEY-----\r\n"
"MHcCAQEEIItF65hjqTd3PzDmyXdEuZgnzci3UVdGUDWj4HgMW3a9oAoGCCqGSM49\r\n"
"AwEHoUQDQgAE+n4RSatGml67pzyg6mVAheq2JuSoZyf1ghFn1hllDLGpqO/UeTzT\r\n"
"Ikq8GYvUKODVSZ1GK1C8rKyrAIrkBiCR7Q==\r\n"
"-----END EC PRIVATE KEY-----\r\n";
