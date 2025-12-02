import React from 'react';
import Layout from '@theme/Layout';

function Register() {
  const handleSubmit = async (event) => {
    event.preventDefault();
    const formData = new FormData(event.target);
    const data = Object.fromEntries(formData.entries());

    const response = await fetch('/api/auth/users/', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        username: data.name,
        email: data.email,
        full_name: data.name,
        password: data.password,
      }),
    });

    if (response.ok) {
      window.location.href = '/login';
    } else {
      alert('Registration failed');
    }
  };

  return (
    <Layout title="Register">
      <div
        style={{
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          height: '50vh',
          flexDirection: 'column',
        }}
      >
        <h1>Register</h1>
        <form onSubmit={handleSubmit}>
          <div style={{ marginBottom: '1rem' }}>
            <label>Name</label>
            <input type="text" name="name" style={{ marginLeft: '0.5rem' }} />
          </div>
          <div style={{ marginBottom: '1rem' }}>
            <label>Email</label>
            <input type="email" name="email" style={{ marginLeft: '0.5rem' }} />
          </div>
          <div style={{ marginBottom: '1rem' }}>
            <label>Password</label>
            <input type="password" name="password" style={{ marginLeft: '0.5rem' }} />
          </div>
          <button type="submit">Register</button>
        </form>
      </div>
    </Layout>
  );
}

export default Register;
